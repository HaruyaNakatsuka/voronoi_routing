from __future__ import annotations
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from collections import defaultdict, deque
import math
import time


# ------------------------------------------------------------
# Helpers (もし既に定義済みなら、ここは消してもOK)
# ------------------------------------------------------------
def build_dist_mat(coords: List[Tuple[float, float]]) -> List[List[float]]:
    n = len(coords)
    dist = [[0.0] * n for _ in range(n)]
    for i in range(n):
        xi, yi = coords[i]
        for j in range(n):
            xj, yj = coords[j]
            dist[i][j] = math.hypot(xj - xi, yj - yi)
    return dist


def build_move_time(service: List[float], dist_mat: List[List[float]]) -> List[List[float]]:
    n = len(service)
    mt = [[0.0] * n for _ in range(n)]
    for i in range(n):
        si = float(service[i])
        for j in range(n):
            mt[i][j] = si + dist_mat[i][j]
    return mt


# ------------------------------------------------------------
# Label
# ------------------------------------------------------------
@dataclass
class _Label:
    dist: float
    time: float
    load: int
    last: int
    prev: Optional["_Label"]  # 復元用ポインタ


def solve_exact_2vehicle_vrp(
    sub_customers: List[Dict],
    sub_PD_pairs: List[Tuple[int, int]],
    start_depots: List[int],
    end_depots: List[int],
    vehicle_capacity: int,
) -> List[List[int]]:
    """
    2車両ぶんの PDPTW（容量・タイムウィンドウ・PD順序）を厳密に解く。

    高速化方針（新DP）：
      - 各車両について「3^m のラベルDP」を1回だけ実行し、
        完了集合（0/2のみ＝open pickup なし）の全 subset に対する最短経路コストを同時に得る。
      - subset の分割は subsetmask と complement の足し算で評価（2^m）。

    pruning:
      (P0) どの状態でも「今すぐ end に戻って due を超える」なら、そのラベルから先の分岐は全て不可能なので展開しない。
      (P1) 0/2-only 状態で「帰還可能ラベルが1つも無い」と確定した subsetmask を infeasible_min(antichain) に登録し、
           それを含む上位 completed-subset を即カットする。

    返り値:
      routes[v] は [start_id, ..., end_id]（ID）
    """

    T0_total = time.perf_counter()

    # -----------------------------
    # 1) バリデーション
    # -----------------------------
    required_keys = {"id", "x", "y", "demand", "ready", "due", "service"}
    for c in sub_customers:
        missing = required_keys - set(c.keys())
        if missing:
            raise ValueError(f"sub_customers に必要キーが不足: missing={missing}, customer={c}")

    if len(start_depots) != 2 or len(end_depots) != 2:
        raise ValueError("start_depots/end_depots は長さ2（2車両）である必要があります。")

    ids = [c["id"] for c in sub_customers]
    if len(ids) != len(set(ids)):
        raise ValueError("sub_customers 内に 'id' の重複があります。")

    id2cust = {c["id"]: c for c in sub_customers}
    for dep_id in start_depots + end_depots:
        if dep_id not in id2cust:
            raise ValueError(f"デポID {dep_id} が sub_customers に存在しません。")

    for p_id, d_id in sub_PD_pairs:
        if p_id not in id2cust or d_id not in id2cust:
            raise ValueError(f"sub_PD_pairs に sub_customers 外のIDが含まれています: ({p_id},{d_id})")

    # -----------------------------------
    # 2) 内部計算用に index 化（ID→index）
    # -----------------------------------
    id2idx = {c["id"]: i for i, c in enumerate(sub_customers)}
    idx2id = [c["id"] for c in sub_customers]
    start_idx = [id2idx[dep_id] for dep_id in start_depots]
    end_idx = [id2idx[dep_id] for dep_id in end_depots]
    pd_idx_pairs: List[Tuple[int, int]] = [(id2idx[p], id2idx[d]) for (p, d) in sub_PD_pairs]
    m = len(pd_idx_pairs)

    # --------------------------------
    # 3) 特殊ケース：PDペアが存在しない
    # --------------------------------
    if m == 0:
        return [[start_depots[0], end_depots[0]], [start_depots[1], end_depots[1]]]

    # -----------------------------
    # 4) 各種必要データの作成
    # -----------------------------
    T0_build = time.perf_counter()
    coords = [(float(c["x"]), float(c["y"])) for c in sub_customers]
    service = [float(c["service"]) for c in sub_customers]
    dist_mat = build_dist_mat(coords)
    move_time = build_move_time(service, dist_mat)
    demand = [int(c["demand"]) for c in sub_customers]
    ready = [float(c["ready"]) for c in sub_customers]
    due = [float(c["due"]) for c in sub_customers]
    FULL = (1 << m) - 1
    same_depot = (start_idx[0] == start_idx[1] and end_idx[0] == end_idx[1])
    T1_build = time.perf_counter()

    print(f"[DBG] build_dist/move_time: {T1_build - T0_build:.3f}s  n={len(sub_customers)}  m={m}")

    # -----------------------------
    # 5) 3進表現の準備
    # -----------------------------
    pow3 = [1] * (m + 1)
    for i in range(m):
        pow3[i + 1] = pow3[i] * 3

    def get_state(m3: int, i: int) -> int:
        return (m3 // pow3[i]) % 3  # 0/1/2

    def inc_state(m3: int, i: int) -> int:
        return m3 + pow3[i]  # 0->1 or 1->2

    def m3_to_subsetmask_if_02_only(m3: int) -> Optional[int]:
        """m3 が 0/2 のみなら subsetmask（bit=1が2）を返す。1が混じるなら None。"""
        subset = 0
        for i in range(m):
            st = get_state(m3, i)
            if st == 1:
                return None
            if st == 2:
                subset |= (1 << i)
        return subset

    def m3_count_012(m3: int) -> Tuple[int, int, int]:
        """(count0, count1, count2) を返す。count1 が open pickup の数。"""
        c0 = c1 = c2 = 0
        for i in range(m):
            st = get_state(m3, i)
            if st == 0:
                c0 += 1
            elif st == 1:
                c1 += 1
            else:
                c2 += 1
        return c0, c1, c2

    # -----------------------------
    # 6) subset prune 用：infeasible_min（antichain）
    # -----------------------------
    def has_infeasible_subset(inf_min: set[int], subsetmask: int) -> Optional[int]:
        for inf in inf_min:
            if (subsetmask & inf) == inf:
                return inf
        return None

    def register_infeasible_min(inf_min: set[int], subsetmask: int) -> None:
        """subsetmask（2値）で「0/2-only 完了集合が帰還不能」と確定したときに登録。antichain維持。"""
        if subsetmask == 0:
            return
        for inf in inf_min:
            if (subsetmask & inf) == inf:
                return
        to_remove = {inf for inf in inf_min if (inf & subsetmask) == subsetmask}
        if to_remove:
            inf_min.difference_update(to_remove)
        inf_min.add(subsetmask)

    # -----------------------------
    # 7) 1車両DP（ラベルDP）で全 subset の最短コストを得る
    # -----------------------------
    def _run_vehicle_dp(v: int) -> Tuple[Dict[int, float], Dict[int, _Label]]:
        s = start_idx[v]
        e = end_idx[v]

        t0 = max(0.0, ready[s])
        if t0 > due[s]:
            raise RuntimeError(f"Vehicle {v}: start depot time window infeasible.")

        # UB（ここは必要なら貪欲等で入れてOK。現状はinfなので distLB は効きにくい）
        ub = float("inf")

        # frontier[(m3,last,load)] = labels
        frontier: Dict[Tuple[int, int, int], List[_Label]] = defaultdict(list)
        mask_to_keys: Dict[int, List[Tuple[int, int, int]]] = defaultdict(list)
        key_seen: set[Tuple[int, int, int]] = set()

        subset_cost: Dict[int, float] = {}
        subset_best_label: Dict[int, _Label] = {}
        infeasible_min: set[int] = set()

        # ---- Debug counters ----
        dbg_t0 = time.perf_counter()
        dbg_m3_popped = 0
        dbg_keys_scanned = 0
        dbg_labels_scanned = 0

        dbg_P0_cut = 0
        dbg_distLB_cut = 0

        dbg_transitions_tried = 0
        dbg_cap_cut = 0
        dbg_tw_cut = 0
        dbg_lb_cut = 0

        dbg_insert_success = 0
        dbg_dominated_drop = 0
        dbg_dominate_remove = 0

        dbg_subset_candidates_seen = 0
        dbg_subset_cost_updates = 0
        dbg_subset_prune_hit = 0

        dbg_P1_checks = 0
        dbg_P1_registered = 0

        # HEAVY 集計（★重要：mm3で数える）
        # m3ごとに「そのm3に属するラベルで何が起きたか」
        heavy_labels: Dict[int, int] = defaultdict(int)
        heavy_trans: Dict[int, int] = defaultdict(int)
        heavy_twcut: Dict[int, int] = defaultdict(int)
        heavy_capcut: Dict[int, int] = defaultdict(int)
        heavy_p0cut: Dict[int, int] = defaultdict(int)

        # completed-subsetの状態メモ（subsetmaskごと）
        # - first_seen_mm3: その subsetmask を初めて見た mm3
        # - any_returnable: 帰還可能ラベルが1つでもあったか（P1判定の結果）
        completed_info: Dict[int, Dict[str, object]] = {}

        # ---- Initial label ----
        init = _Label(dist=0.0, time=t0, load=0, last=s, prev=None)
        init_key = (0, s, 0)
        frontier[init_key].append(init)
        mask_to_keys[0].append(init_key)
        key_seen.add(init_key)

        def insert_label(key: Tuple[int, int, int], cand: _Label) -> None:
            nonlocal dbg_insert_success, dbg_dominated_drop, dbg_dominate_remove
            lst = frontier[key]

            # 既存が cand を支配 → 捨て
            for lab in lst:
                if (lab.dist <= cand.dist) and (lab.time <= cand.time) and (lab.load <= cand.load):
                    dbg_dominated_drop += 1
                    return

            # cand が既存を支配 → 除去
            new_lst: List[_Label] = []
            removed = 0
            for lab in lst:
                if (cand.dist <= lab.dist) and (cand.time <= lab.time) and (cand.load <= lab.load):
                    removed += 1
                    continue
                new_lst.append(lab)
            if removed:
                dbg_dominate_remove += removed

            new_lst.append(cand)
            frontier[key] = new_lst
            dbg_insert_success += 1

            if key not in key_seen:
                key_seen.add(key)
                mask_to_keys[key[0]].append(key)

        pending = deque([0])
        pending_seen = {0}

        # 進捗printの頻度（m3が大きい問題では頻繁すぎると邪魔なので）
        PROGRESS_EVERY = 2000

        while pending:
            m3 = pending.popleft()
            dbg_m3_popped += 1

            if (dbg_m3_popped % PROGRESS_EVERY) == 0:
                # 大まかな総量（高速に取るため概算）
                keys_total = sum(len(vv) for vv in mask_to_keys.values())
                labels_total = sum(len(frontier[k]) for k in frontier.keys())
                ub_str = "inf" if ub == float("inf") else f"{ub:.6f}"
                print(
                    f"[DBG][v={v}] popped_m3={dbg_m3_popped} pending={len(pending)} seen_m3={len(pending_seen)} "
                    f"keys={keys_total} labels~={labels_total} P0_cut={dbg_P0_cut} "
                    f"ins_succ={dbg_insert_success} dom_drop={dbg_dominated_drop} "
                    f"subCost={len(subset_cost)} infMin={len(infeasible_min)} ub={ub_str}"
                )

            keys = mask_to_keys.get(m3, [])
            for (mm3, last, load) in keys:
                dbg_keys_scanned += 1
                lab_list = list(frontier[(mm3, last, load)])
                for lab in lab_list:
                    dbg_labels_scanned += 1

                    # ★HEAVY：このラベルは mm3 に属する
                    heavy_labels[mm3] += 1

                    dist_sofar = lab.dist
                    time_sofar = lab.time
                    load_sofar = lab.load

                    # (P0) 今すぐendに戻って due 超過なら、このラベルから先は絶対無理
                    arrive_end = time_sofar + move_time[last][e]
                    t_end = max(arrive_end, ready[e])
                    if t_end > due[e]:
                        dbg_P0_cut += 1
                        heavy_p0cut[mm3] += 1
                        continue

                    # 距離LB（ub が finite の時だけ効く）
                    if dist_sofar + dist_mat[last][e] >= ub:
                        dbg_distLB_cut += 1
                        continue

                    # 0/2-only なら subsetmask を確定して記録
                    subsetmask = m3_to_subsetmask_if_02_only(mm3)
                    if subsetmask is not None:
                        dbg_subset_candidates_seen += 1

                        if subsetmask not in completed_info:
                            completed_info[subsetmask] = {
                                "first_seen_mm3": mm3,
                                "first_seen_last": last,
                                "first_seen_load": load_sofar,
                                "first_seen_time": time_sofar,
                                "any_returnable": None,
                                "registered_P1": False,
                            }

                        bad = has_infeasible_subset(infeasible_min, subsetmask)
                        if bad is not None:
                            dbg_subset_prune_hit += 1
                            # completed-subsetを含む上位completed-subsetを落としてる
                            continue

                        # ここまで来た時点で P0 を通っているので帰還は due 内に可能
                        total = dist_sofar + dist_mat[last][e]
                        prev = subset_cost.get(subsetmask)
                        if (prev is None) or (total < prev):
                            subset_cost[subsetmask] = total
                            subset_best_label[subsetmask] = lab
                            dbg_subset_cost_updates += 1
                            if subsetmask == FULL and total < ub:
                                ub = total

                    # 遷移（各ペアについて 0->pickup, 1->delivery）
                    for i, (p, d) in enumerate(pd_idx_pairs):
                        st = get_state(mm3, i)
                        if st == 2:
                            continue

                        nxt = p if st == 0 else d
                        dbg_transitions_tried += 1
                        heavy_trans[mm3] += 1

                        new_load = load_sofar + demand[nxt]
                        if new_load < 0 or new_load > vehicle_capacity:
                            dbg_cap_cut += 1
                            heavy_capcut[mm3] += 1
                            continue

                        arrive = time_sofar + move_time[last][nxt]
                        new_time = max(arrive, ready[nxt])
                        if new_time > due[nxt]:
                            dbg_tw_cut += 1
                            heavy_twcut[mm3] += 1
                            continue

                        new_dist = dist_sofar + dist_mat[last][nxt]
                        if new_dist + dist_mat[nxt][e] >= ub:
                            dbg_lb_cut += 1
                            continue

                        new_m3 = inc_state(mm3, i)
                        new_key = (new_m3, nxt, new_load)
                        new_lab = _Label(dist=new_dist, time=new_time, load=new_load, last=nxt, prev=lab)

                        before = dbg_insert_success
                        insert_label(new_key, new_lab)
                        # insert_label が “支配されて drop” なら insert_success は増えない
                        # dbg_insert_success の増分は insert_label 内で数える

                        if new_m3 not in pending_seen:
                            pending_seen.add(new_m3)
                            pending.append(new_m3)

            # (P1) 0/2-only 状態 mm3 について、帰還可能ラベルが1つも無いなら infeasible_min に登録
            #      ただし、mm3 はこのループの対象 m3 と必ずしも一致しないので、
            #      P1チェックは「poppedした m3 自体」に対してやる（この仕様は元コード通り）。
            subsetmask = m3_to_subsetmask_if_02_only(m3)
            if subsetmask is not None:
                dbg_P1_checks += 1
                if has_infeasible_subset(infeasible_min, subsetmask) is None:
                    any_returnable = False
                    for (mm3x, lastx, loadx) in mask_to_keys.get(m3, []):
                        for labx in frontier[(mm3x, lastx, loadx)]:
                            arrive_end = labx.time + move_time[lastx][e]
                            t_end = max(arrive_end, ready[e])
                            if t_end <= due[e]:
                                any_returnable = True
                                break
                        if any_returnable:
                            break

                    # completed_info にも反映
                    if subsetmask in completed_info:
                        completed_info[subsetmask]["any_returnable"] = any_returnable

                    if not any_returnable:
                        register_infeasible_min(infeasible_min, subsetmask)
                        dbg_P1_registered += 1
                        if subsetmask in completed_info:
                            completed_info[subsetmask]["registered_P1"] = True

        # ---- Summary ----
        dbg_t1 = time.perf_counter()
        print(f"[DBG][v={v}] ===================== DP SUMMARY =====================")
        print(f"[DBG][v={v}] dp_time = {dbg_t1 - dbg_t0:.3f}s")
        print(f"[DBG][v={v}] m3_popped={dbg_m3_popped} seen_m3={len(pending_seen)}")
        print(f"[DBG][v={v}] keys_scanned={dbg_keys_scanned} labels_scanned={dbg_labels_scanned}")
        print(f"[DBG][v={v}] total_keys={len(frontier)} total_labels={sum(len(frontier[k]) for k in frontier.keys())}")
        print(f"[DBG][v={v}] P0_cut(return_impossible)={dbg_P0_cut}")
        ub_str = "inf" if ub == float("inf") else f"{ub:.6f}"
        print(f"[DBG][v={v}] distLB_cut={dbg_distLB_cut} (ub={ub_str})")
        print(f"[DBG][v={v}] subset_candidates_seen={dbg_subset_candidates_seen} subset_cost_size={len(subset_cost)}")
        print(f"[DBG][v={v}] subset_cost_updates={dbg_subset_cost_updates}")
        print(f"[DBG][v={v}] subset_prune_hit={dbg_subset_prune_hit} infeasible_min_size={len(infeasible_min)}")
        print(f"[DBG][v={v}] P1_checks={dbg_P1_checks} P1_registered={dbg_P1_registered}")
        print(f"[DBG][v={v}] transitions_tried={dbg_transitions_tried}")
        print(f"[DBG][v={v}]   cap_cut={dbg_cap_cut} tw_cut={dbg_tw_cut} lb_cut={dbg_lb_cut}")
        print(f"[DBG][v={v}] insert_success={dbg_insert_success} dominated_drop={dbg_dominated_drop} dominate_remove={dbg_dominate_remove}")
        print(f"[DBG][v={v}] =======================================================")

        # ---- HEAVY m3 RANKING (FIXED: by mm3) ----
        # 重さ指標：labels + trans でまず並べる（必要なら好きに変えてOK）
        heavy_scores = []
        for mm3, labcnt in heavy_labels.items():
            tr = heavy_trans.get(mm3, 0)
            twc = heavy_twcut.get(mm3, 0)
            capc = heavy_capcut.get(mm3, 0)
            p0c = heavy_p0cut.get(mm3, 0)
            c0, c1, c2 = m3_count_012(mm3)  # c1=open pickup
            heavy_scores.append((labcnt + tr, labcnt, tr, twc, capc, p0c, mm3, c0, c1, c2))

        heavy_scores.sort(reverse=True)

        TOPK = 30
        print(f"[DBG][v={v}] ===== HEAVY m3 STATES (by mm3; with open-pickup count) =====")
        for rank, (score, labcnt, tr, twc, capc, p0c, mm3, c0, c1, c2) in enumerate(heavy_scores[:TOPK], 1):
            subset02 = (c1 == 0)
            print(
                f"[DBG][v={v}] #{rank:02d} mm3={mm3}  score={score}  labels={labcnt}  trans={tr}  "
                f"TWcut={twc}  CAPcut={capc}  P0cut={p0c}  "
                f"(0,1,2)=({c0},{c1},{c2})  subset02={subset02}"
            )
        print(f"[DBG][v={v}] ===========================================================")

        # ---- COMPLETED SUBSET STATUS (0/2-only) ----
        # 「completed-subset に落ちたところで何が起きているか」を一覧化
        #  - returnable がほぼ True なら P1 は効かない（旧方式の subset prune と差が出る可能性）
        #  - returnable False があるのに registered_P1 が少ないなら、判定やタイミングに問題がある可能性
        comp_items = []
        for sm, info in completed_info.items():
            first_mm3 = int(info["first_seen_mm3"])
            c0, c1, c2 = m3_count_012(first_mm3)
            # c1==0 のはずだが念のため
            comp_items.append((
                sm,
                int(info["first_seen_mm3"]),
                int(info["first_seen_last"]),
                int(info["first_seen_load"]),
                float(info["first_seen_time"]),
                info["any_returnable"],
                bool(info["registered_P1"]),
                c2,  # size=完了ペア数
            ))
        comp_items.sort(key=lambda x: (-x[7], x[0]))  # 大きいsubsetから

        print(f"[DBG][v={v}] ===== COMPLETED SUBSET STATUS (0/2-only) =====")
        print(f"[DBG][v={v}] shown up to 30 of {len(comp_items)} subsets")
        for sm, first_mm3, last, loadx, timex, any_ret, reg, size2 in comp_items[:30]:
            print(
                f"[DBG][v={v}] subsetmask={bin(sm)} size={size2} "
                f"first_mm3={first_mm3} last={last} load={loadx} time={timex:.3f} "
                f"any_returnable={any_ret} registered_P1={reg}"
            )
        print(f"[DBG][v={v}] ============================================")

        return subset_cost, subset_best_label

    # -----------------------------
    # 8) DP実行（車両0、車両1）
    # -----------------------------
    print(">車両0と車両1の2GAT検証開始")
    print(f"車両0 start/end={start_depots[0]}->{end_depots[0]}")
    print(f"車両1 start/end={start_depots[1]}->{end_depots[1]}")

    T0_v0 = time.perf_counter()
    cost0, bestlab0 = _run_vehicle_dp(0)
    T1_v0 = time.perf_counter()
    print(f"[DBG] vehicle0 DP done: {T1_v0 - T0_v0:.3f}s  subsets={len(cost0)}")

    if same_depot:
        cost1, bestlab1 = cost0, bestlab0
        print("[DBG] same_depot=True => reuse DP results for vehicle1")
    else:
        T0_v1 = time.perf_counter()
        cost1, bestlab1 = _run_vehicle_dp(1)
        T1_v1 = time.perf_counter()
        print(f"[DBG] vehicle1 DP done: {T1_v1 - T0_v1:.3f}s  subsets={len(cost1)}")

    # -----------------------------
    # 9) 2車両の最適分割（2^m）
    # -----------------------------
    T0_split = time.perf_counter()
    best_total = float("inf")
    best_subset = None
    feasible_pairs = 0

    for subset in range(FULL + 1):
        comp = FULL ^ subset
        c0 = cost0.get(subset)
        if c0 is None:
            continue
        c1 = cost1.get(comp)
        if c1 is None:
            continue
        feasible_pairs += 1
        total = c0 + c1
        if total < best_total:
            best_total = total
            best_subset = subset

    T1_split = time.perf_counter()
    print(f"[DBG] split eval time: {T1_split - T0_split:.3f}s")
    print(f"[DBG] split feasible_pairs={feasible_pairs} / {FULL+1}")
    if best_subset is not None:
        print(f"[DBG] best_total={best_total} best_subset(bin)={bin(best_subset)}")

    if best_subset is None:
        raise RuntimeError("2車両の厳密解が見つかりません（入力が不可能制約の可能性）。")

    # -----------------------------
    # 10) 経路復元（subsetmask → ラベルを辿る）
    # -----------------------------
    def _reconstruct_route(v: int, subsetmask: int, bestlab: Dict[int, _Label]) -> List[int]:
        s = start_idx[v]
        e = end_idx[v]
        lab = bestlab[subsetmask]

        rev = []
        cur = lab
        while cur is not None:
            rev.append(cur.last)
            cur = cur.prev
        route = list(reversed(rev))
        if not route or route[0] != s:
            route = [s] + [x for x in route if x not in (s, e)]
        if route[-1] != e:
            route.append(e)
        return route

    r0_idx = _reconstruct_route(0, best_subset, bestlab0)
    r1_idx = _reconstruct_route(1, FULL ^ best_subset, bestlab1)

    # -----------------------------
    # 11) index → ID に直して返す
    # -----------------------------
    r0_id = [idx2id[i] for i in r0_idx]
    r1_id = [idx2id[i] for i in r1_idx]

    T1_total = time.perf_counter()
    print(f"[DBG] TOTAL solve_exact_2vehicle_vrp time: {T1_total - T0_total:.3f}s")

    return [r0_id, r1_id]
