from __future__ import annotations
from typing import Dict, List, Tuple, Optional, TypeAlias
from collections import defaultdict
import itertools
import math


def build_dist_mat(coords: List[Tuple[float, float]]) -> List[List[float]]:
    """coords[index]=(x,y) に対する距離行列（float）を作る"""
    n = len(coords)
    dist_mat = [[0.0] * n for _ in range(n)]
    for i in range(n):
        xi, yi = coords[i]
        row = dist_mat[i]
        for j in range(n):
            xj, yj = coords[j]
            row[j] = math.hypot(xj - xi, yj - yi)
    return dist_mat


def build_move_time(service: List[float], dist_mat: List[List[float]]) -> List[List[float]]:
    """
    move_time[i][j] = service[i] + dist_mat[i][j]
    （あなたの定義：出発ノードiでのサービス後に移動する所要時間）
    """
    n = len(service)
    move_time = [[0.0] * n for _ in range(n)]
    for i in range(n):
        si = float(service[i])
        row_i = move_time[i]
        dist_row = dist_mat[i]
        for j in range(n):
            row_i[j] = si + dist_row[j]
    return move_time


def route_cost_idx(route_idx: List[int], dist_mat: List[List[float]]) -> float:
    """
    ルート（index列）の総移動距離を返す。
    route_idx: [start_idx, ..., end_idx]
    dist_mat: 事前計算した距離行列（float）
    """
    if route_idx is None or len(route_idx) < 2:
        return 0.0
    s = 0.0
    for a, b in zip(route_idx, route_idx[1:]):
        s += dist_mat[a][b]
    return s


# ============================================================
# 1台の PDPTW を「厳密に」解く（ラベリング/DP）
# - 目的: 距離最小
# - 制約: 容量 / タイムウィンドウ / PD順序（pickup -> delivery）
# - 入力は Index 基準
# ============================================================
def solve_1vehicle_pdptw_exact_idx(
    sub_customers: List[Dict],
    start_depot_idx: int,
    end_depot_idx: int,
    vehicle_capacity: int,
    dist_mat: List[List[float]],
    move_time: List[List[float]],
    *,
    vehicle_km1_route_list: Optional[List[List[int]]] = None,
    added_pair: Optional[Tuple[int, int]] = None,
) -> Tuple[Optional[List[int]], List[List[int]]]:
    """
      入力:
        - vehicle_km1_route_list: サイズ(k-1)の feasible ルート集合（各要素は [start,...,end] の index 列）
        - added_pair: (pickup_idx, delivery_idx) を 1つ追加する

      出力:
        - best_route: 最短距離の feasible ルート（なければ None）
        - vehicle_k_route_list: 発見した feasible ルートの全列挙（重複は除去）

    注意:
      - 上界による枝刈りは行わない（真にinfeasibleな解のみを破棄する）
      - k=0 の場合は vehicle_km1_route_list/added_pair は不要（デポ直行を判定して返す）
    """

    # =========================================================
    # DEBUG SWITCHES
    # =========================================================
    DBG = True
    DBG_EVERY_BASE = 0              # 0:出さない / 1以上:その間隔でbaseごとの統計を出す（例:50なら50本ごと）
    DBG_SHOW_SAMPLES = 5            # 代表例を何件出すか
    # =========================================================

    # ---- 前処理（配列で持つ：index基準） ----
    demand = [int(c["demand"]) for c in sub_customers]
    ready = [float(c["ready"]) for c in sub_customers]
    due = [float(c["due"]) for c in sub_customers]
    service = [float(c["service"]) for c in sub_customers]  # move_timeがserviceを含んでも終端判定等で使うことがある

    def route_cost(route: List[int]) -> float:
        s = 0.0
        for i in range(len(route) - 1):
            s += float(dist_mat[route[i]][route[i + 1]])
        return s

    def check_feasible(route: List[int]) -> bool:
        """
        route = [start, ..., end] を全走査して
          - 容量制約
          - TW制約
        を判定する（厳密）。
        time 更新:
          arrive = time + move_time[last][nxt]
          time = max(arrive, ready[nxt])
        """
        if not route or route[0] != start_depot_idx or route[-1] != end_depot_idx:
            return False

        t = max(0.0, ready[start_depot_idx])
        if t > due[start_depot_idx]:
            return False

        load = 0
        for i in range(1, len(route)):
            prev = route[i - 1]
            nxt = route[i]

            arrive = t + float(move_time[prev][nxt])
            t = max(arrive, ready[nxt])
            if t > due[nxt]:
                return False

            load += int(demand[nxt])
            if load < 0 or load > vehicle_capacity:
                return False

        return True

    # ---- k=0: added_pair/km1なしならデポ直行 ----
    if (vehicle_km1_route_list is None) and (added_pair is None):
        route0 = [start_depot_idx, end_depot_idx]
        if check_feasible(route0):
            if DBG:
                print("[DBG][k=0] direct depot route feasible -> return 1 route")
            return route0, [route0]
        raise RuntimeError(
            "[solve_1vehicle_pdptw_exact_idx] "
            "Direct depot route [start_depot -> end_depot] is infeasible. "
            "This is unexpected; check depot time windows/service time/move_time definitions."
        )

    # ---- 入力チェック（データ整合性エラー） ----
    if vehicle_km1_route_list is None:
        raise ValueError(
            "[solve_1vehicle_pdptw_exact_idx] "
            "vehicle_km1_route_list is missing, so the incremental (k-1 -> k) search cannot be executed."
        )
    if added_pair is None:
        raise ValueError(
            "[solve_1vehicle_pdptw_exact_idx] "
            "added_pair is not specified, so the incremental (k-1 -> k) search cannot be executed."
        )

    p, d = added_pair
    if p == d:
        raise ValueError(
            "[solve_1vehicle_pdptw_exact_idx] invalid added_pair: pickup == delivery "
            f"(p=d={p})."
        )
    if (p == start_depot_idx) or (p == end_depot_idx) or (d == start_depot_idx) or (d == end_depot_idx):
        raise ValueError(
            "[solve_1vehicle_pdptw_exact_idx] invalid added_pair: depot index appears in pickup/delivery. "
            f"added_pair=(p={p}, d={d}), start_depot_idx={start_depot_idx}, end_depot_idx={end_depot_idx}."
        )

    # =========================================================
    # DEBUG counters (hotspot切り分け用)
    # =========================================================
    km1_n = len(vehicle_km1_route_list)
    p_slots_total = 0
    d_slots_total = 0
    gen_candidates_total = 0
    feas_checks_total = 0
    feas_true_total = 0
    uniq_updates_total = 0
    best_updates_total = 0
    bad_base_skipped = 0

    # 代表例（どこで落ちてるか見る用）
    sample_feasible_routes: List[List[int]] = []
    sample_base_stats: List[Tuple[int, int, int, int]] = []  # (base_len, gen, feas_true, uniq_added)

    if DBG:
        # base長から、おおざっぱな候補総数を見積もる（実測は下で集計）
        lens = [len(b) for b in vehicle_km1_route_list if b]
        if lens:
            avgL = sum(lens) / len(lens)
            # base長=L のとき候補数は (L-1)*((L)*(L-1)/2) 程度（p_pos*(d_pos)の二重）
            # 厳密ではないが目安として出す
            print(
                f"[DBG] km1_routes={km1_n}, added_pair=(p={p}, d={d}), "
                f"avg_base_len={avgL:.2f}, example_base_len={lens[0]}"
            )

    # ---- 最良解 ----
    best_route: Optional[List[int]] = None
    best_cost = float("inf")

    # ルート重複除去（tuple化して set/dict 管理）
    uniq: Dict[Tuple[int, ...], float] = {}

    for base_i, base in enumerate(vehicle_km1_route_list):
        if not base or base[0] != start_depot_idx or base[-1] != end_depot_idx:
            bad_base_skipped += 1
            continue

        L = len(base)
        base_gen = 0
        base_feas = 0
        base_uniq_added = 0

        # p挿入スロット i=1..L-1
        for p_pos in range(1, L):
            p_slots_total += 1

            # listコピーが重い可能性があるので、どのくらい回っているかをまず計測
            route_p = base[:p_pos] + [p] + base[p_pos:]
            Lp = L + 1

            # d挿入スロット d_pos = p_pos+1 .. Lp-1
            for d_pos in range(p_pos + 1, Lp):
                d_slots_total += 1
                gen_candidates_total += 1
                base_gen += 1

                new_route = route_p[:d_pos] + [d] + route_p[d_pos:]

                feas_checks_total += 1
                if not check_feasible(new_route):
                    continue

                feas_true_total += 1
                base_feas += 1

                key = tuple(new_route)
                c = route_cost(new_route)

                prev = uniq.get(key)
                if prev is None or c < prev:
                    if prev is None:
                        base_uniq_added += 1
                    uniq[key] = c
                    uniq_updates_total += 1

                if c < best_cost:
                    best_cost = c
                    best_route = list(new_route)
                    best_updates_total += 1

                # 代表例だけ保存
                if DBG and len(sample_feasible_routes) < DBG_SHOW_SAMPLES:
                    sample_feasible_routes.append(list(new_route))

        if DBG and DBG_EVERY_BASE and (base_i % DBG_EVERY_BASE == 0):
            sample_base_stats.append((L, base_gen, base_feas, base_uniq_added))
            print(
                f"[DBG][base {base_i}/{km1_n}] L={L} generated={base_gen} "
                f"feasible={base_feas} uniq_added={base_uniq_added} "
                f"uniq_size_now={len(uniq)}"
            )

    vehicle_k_route_list = [list(r) for r in uniq.keys()]

    if DBG:
        print("[DBG] --------------------------------------------------")
        print(f"[DBG] km1_routes={km1_n}, bad_base_skipped={bad_base_skipped}")
        print(f"[DBG] p_slots_total={p_slots_total}, d_slots_total={d_slots_total}")
        print(f"[DBG] generated_candidates_total={gen_candidates_total}")
        print(f"[DBG] feasible_checks_total={feas_checks_total}")
        print(f"[DBG] feasible_true_total={feas_true_total}")
        print(f"[DBG] uniq_size_final={len(uniq)} (returned vehicle_k_route_list size)")
        print(f"[DBG] uniq_updates_total={uniq_updates_total} (incl. overwrite by lower cost)")
        print(f"[DBG] best_updates_total={best_updates_total}")
        if best_route is None:
            print("[DBG] best_route=None (no feasible route found)")
        else:
            print(f"[DBG] best_cost={best_cost:.6f}, best_route_len={len(best_route)}")
        if sample_feasible_routes:
            print(f"[DBG] sample feasible routes (up to {DBG_SHOW_SAMPLES}):")
            for r in sample_feasible_routes:
                print(f"      {r}")
        if sample_base_stats and not DBG_EVERY_BASE:
            # DBG_EVERY_BASE==0なら出ないのでここは通常通らない
            pass

    return best_route, vehicle_k_route_list




def solve_exact_2vehicle_vrp(
    sub_customers: List[Dict],
    sub_PD_pairs: List[Tuple[int, int]],
    start_depots: List[int],
    end_depots: List[int],
    vehicle_capacity: int,
) -> List[List[int]]:
    """
    2車両ぶんの PDPTW（容量・タイムウィンドウ・PD順序）を「厳密に」解くための関数。

    入力（重要：すべて "ID" 基準）
    - sub_customers:
        ノード辞書のリスト。各辞書は少なくとも以下を含む想定:
        {
          'id': int, 'x': float, 'y': float,
          'demand': int,
          'ready': int, 'due': int,
          'service': int,
          ...（pickup_index/delivery_indexがあっても本関数ではsub_PD_pairsを使う）
        }
    - sub_PD_pairs:
        (pickup_id, delivery_id) のリスト。ここに入っている数値も "ID"。
    - start_depots / end_depots:
        長さ2のリスト。各車両の出発/到着デポの "ID"。
        例: [200, 200]
    - vehicle_capacity:
        車両容量（2台とも同じ前提）

    出力（ID基準）
    - routes: 長さ2の2次元リスト。
        routes[v] は [start_id, ..., end_id] で、訪問順序が入る。
        例: [[0, 113, ..., 0], [0, 45, ..., 0]]

    目的関数
    - Min( 2台の合計距離（ユークリッド） ）
    """

    # =========================================================
    # VERIFY SWITCHES
    # =========================================================
    VERIFY = False                 # まとめてON/OFF
    VERIFY_VEH_FEAS = True         # (A) veh_feas と solver の一致検証
    VERIFY_TRUE_OPT = True         # (B) 2車両の真の最適と best_total 一致検証
    # =========================================================

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
    pd_idx_pairs: List[Tuple[int, int]] = [(id2idx[p], id2idx[d]) for p, d in sub_PD_pairs]

    # --------------------------------
    # 3) 特殊ケース：PDペアが存在しない
    # --------------------------------
    m = len(pd_idx_pairs)
    if m == 0:
        return [[start_depots[0], end_depots[0]], [start_depots[1], end_depots[1]]]

    # -----------------------------
    # 4) 各種必要データの作成
    # -----------------------------
    coords = [(float(c["x"]), float(c["y"])) for c in sub_customers]
    service = [float(c["service"]) for c in sub_customers]
    dist_mat = build_dist_mat(coords)
    move_time = build_move_time(service, dist_mat)

    FULL = (1 << m) - 1
    half = m / 2.0
    same_depot = (start_idx[0] == start_idx[1] and end_idx[0] == end_idx[1])

    # -----------------------------
    # 5) 車両ごとの各種キャッシュ
    # -----------------------------
    # veh_feas: 「車両vがmaskを単独で回れるか？」= solver一致対象
    vehicle_feasible: List[Dict[int, bool]] = [{}, {}]
    vehicle_cost: List[Dict[int, float]] = [{}, {}]
    vehicle_route: List[Dict[int, List[int]]] = [{}, {}]

    # vehicle_routes_all：各maskに対して feasible な経路をすべて保存
    # vehicle_routes_all[v][mask] = List[route]
    vehicle_routes_all: List[Dict[int, List[List[int]]]] = [{}, {}]

    # useful: 「2車両探索でこのmask割当が意味あるか？」（comp_pruneでFalseになり得る）
    useful: List[Dict[int, bool]] = [{}, {}]

    # 既知 infeasible の「極小集合（antichain）」だけを保持し、
    #      部分集合チェックで上位集合を枝刈りするための集合
    infeasible_min: List[set[int]] = [set(), set()]

    # subset_prune の根拠となった下位集合のmaskを記録
    subset_pruned_bad: List[Dict[int, int]] = [{}, {}]  # mask -> bad

    # -----------------------------
    # 6) ユーティリティ
    # -----------------------------
    def build_pairs_from_mask(mask: int) -> List[Tuple[int, int]]:
        return [pd_idx_pairs[k] for k in range(m) if (mask >> k) & 1]

    def mask_to_pairs_id(mask: int) -> List[Tuple[int, int]]:
        return [(idx2id[p], idx2id[d]) for (p, d) in build_pairs_from_mask(mask)]

    def has_infeasible_subset(v: int, mask: int) -> int | None:
        """含んでいる infeasible subset を1つ返す（なければNone）"""
        for inf in infeasible_min[v]:
            if (mask & inf) == inf:
                return inf
        return None

    def register_infeasible(v: int, mask: int) -> None:
        """solverでNoneだったときだけ呼ぶ（重要：comp_pruneでは絶対呼ばない）"""
        if mask == 0:
            return
        for inf in infeasible_min[v]:
            if (mask & inf) == inf:
                return
        to_remove = {inf for inf in infeasible_min[v] if (inf & mask) == mask}
        if to_remove:
            infeasible_min[v].difference_update(to_remove)
        infeasible_min[v].add(mask)

    def _leftmost_one_bit_index(mask: int) -> int:
        """mask の最左(=最小index)の 1-bit の位置を返す。mask>0 前提。"""
        lsb = mask & -mask
        return (lsb.bit_length() - 1)

    def solve_vehicle(v: int, mask: int) -> None:
        """vehicle_feasible/vehicle_cost/vehicle_route/vehicle_routes_all を埋める（未計算なら計算）"""
        if mask in vehicle_feasible[v]:
            return

        # ---- k=0 はデポ直行を vehicle_k_route_list として確定させる ----
        if mask == 0:
            best_route, route_list = solve_1vehicle_pdptw_exact_idx(
                sub_customers=sub_customers,
                start_depot_idx=start_idx[v],
                end_depot_idx=end_idx[v],
                vehicle_capacity=vehicle_capacity,
                dist_mat=dist_mat,
                move_time=move_time,
                vehicle_km1_route_list=None,
                added_pair=None,
            )
            if best_route is None:
                vehicle_feasible[v][0] = False
                useful[v][0] = False
                vehicle_routes_all[v][0] = []
                raise RuntimeError(f"Empty assignment (mask=0) infeasible for vehicle {v}")
            c = route_cost_idx(best_route, dist_mat)
            vehicle_feasible[v][0] = True
            useful[v][0] = True
            vehicle_route[v][0] = best_route
            vehicle_cost[v][0] = c
            vehicle_routes_all[v][0] = route_list
            return

        # ---- k>=1: 増分挿入（base_mask + added_pair） ----
        bit = _leftmost_one_bit_index(mask)
        base_mask = mask ^ (1 << bit)
        added_pair = pd_idx_pairs[bit]

        # base を先に解いて route_list を確保
        solve_vehicle(v, base_mask)
        if not vehicle_feasible[v].get(base_mask, False):
            # 単調性により base infeasible => mask infeasible
            vehicle_feasible[v][mask] = False
            useful[v][mask] = False
            vehicle_routes_all[v][mask] = []
            register_infeasible(v, mask)
            return

        vehicle_km1_route_list = vehicle_routes_all[v].get(base_mask)
        if vehicle_km1_route_list is None:
            # ここに来るのは実装上おかしい（base を解いたなら route_list があるはず）
            raise RuntimeError(f"vehicle_routes_all[{v}][{base_mask}] is missing.")

        best_route, route_list = solve_1vehicle_pdptw_exact_idx(
            sub_customers=sub_customers,
            start_depot_idx=start_idx[v],
            end_depot_idx=end_idx[v],
            vehicle_capacity=vehicle_capacity,
            dist_mat=dist_mat,
            move_time=move_time,
            vehicle_km1_route_list=vehicle_km1_route_list,
            added_pair=added_pair,
        )

        if best_route is None:
            vehicle_feasible[v][mask] = False
            useful[v][mask] = False
            vehicle_routes_all[v][mask] = []
            register_infeasible(v, mask)
        else:
            c = route_cost_idx(best_route, dist_mat)
            vehicle_feasible[v][mask] = True
            useful[v][mask] = True
            vehicle_route[v][mask] = best_route
            vehicle_cost[v][mask] = c
            vehicle_routes_all[v][mask] = route_list

    # ----------------------------------------
    # 7-1) 探索本体：PD割り当て数=0の場合の検証
    # ----------------------------------------
    for v in (0, 1):
        solve_vehicle(v, 0)

    # -----------------------------------------
    # 7-2) 探索本体：PD割り当て数>=1の場合の検証
    # -----------------------------------------
        # ここでは「PDペアの割当」を bitmask（長さm）で表す。
    #   - mask の bit=1 になっているPDペア集合を “車両0” が担当
    #   - comp = FULL ^ mask（補集合）を “車両1” が担当
    #
    # ■検証順序（重要）
    #   PDペア数 k の昇順（k=1,2,...,m）で、k個のペアを選ぶ全組合せを列挙し mask を作る。
    #   この順序により、任意の mask の「真部分集合」は必ず先に評価済みになる。
    #
    # ■枝刈り戦略
    #   1) subset prune（単調性に基づく枝刈り）
    #      すでに “infeasible と確定した集合” を含む（=部分集合として持つ）mask は、
    #      今回の問題設定では上位集合も必ず infeasible とみなせるためスキップできる。
    #      具体的には、infeasible_min（infeasibleの極小集合＝antichain）だけを保持し、
    #      ∃inf∈infeasible_min s.t. (mask & inf)==inf なら枝刈りする。
    #
    #   2) comp prune（2車両性に基づく枝刈り）
    #      k > m/2 の領域では補集合 comp のペア数は < m/2 なので、comp は既に評価済みのはず。
    #      もし comp が 1台で infeasible なら、mask が 1台で feasible でも 2車両解になり得ないため、
    #      mask の評価（solve_1vehicle呼び出し）自体を省略できる。
    #
    # ■妥当性（なぜ落としてよいか）
    #   - subset prune は「infeasible subset ⇒ infeasible superset」という単調性に基づく。
    #     （この性質が成り立つ前提のもとで、厳密性を損なわず探索数だけ減らす。）
    #   - comp prune は「2車両解には mask と comp の両方が1台で feasible である必要がある」という
    #     必要条件に基づくため、厳密性を損なわない。
    #
    # なお、best_total の更新は k >= m/2 の段階で行う（mask と comp の両方が揃う領域のため）。
    best_total = float("inf")
    best_r0_idx = None
    best_r1_idx = None

    for k in range(1, m + 1):
        print(f"PDペア数 = {k}ペアを探索中")
        feasible_masks_k: List[List[int]] = [[], []]

        for comb in itertools.combinations(range(m), k):
            mask = 0
            for bitpos in comb:
                mask |= (1 << bitpos)
            comp = FULL ^ mask

            if same_depot:
                v = 0
                bad = has_infeasible_subset(v, mask)
                if bad is not None:
                    vehicle_feasible[0][mask] = False
                    vehicle_feasible[1][mask] = False
                    useful[0][mask] = False
                    useful[1][mask] = False
                    vehicle_routes_all[0][mask] = []
                    vehicle_routes_all[1][mask] = []
                    subset_pruned_bad[0][mask] = bad
                    subset_pruned_bad[1][mask] = bad
                    continue

                if k > half:
                    solve_vehicle(0, comp)
                    if not vehicle_feasible[0][comp]:
                        useful[0][mask] = False
                        useful[1][mask] = False
                        continue

                solve_vehicle(0, mask)
                # same_depotなのでコピー
                vehicle_feasible[1][mask] = vehicle_feasible[0][mask]
                if mask in vehicle_cost[0]:
                    vehicle_cost[1][mask] = vehicle_cost[0][mask]
                if mask in vehicle_route[0]:
                    vehicle_route[1][mask] = vehicle_route[0][mask]
                if mask in vehicle_routes_all[0]:
                    vehicle_routes_all[1][mask] = vehicle_routes_all[0][mask]

                if vehicle_feasible[0].get(mask, False):
                    feasible_masks_k[0].append(mask)

            else:
                for v in (0, 1):
                    other = 1 - v

                    bad = has_infeasible_subset(v, mask)
                    if bad is not None:
                        vehicle_feasible[v][mask] = False
                        useful[v][mask] = False
                        vehicle_routes_all[v][mask] = []
                        subset_pruned_bad[v][mask] = bad
                        continue

                    if k > half:
                        solve_vehicle(other, comp)
                        if not vehicle_feasible[other][comp]:
                            useful[v][mask] = False
                            continue

                    solve_vehicle(v, mask)
                    if vehicle_feasible[v].get(mask, False):
                        feasible_masks_k[v].append(mask)

        # total 更新（k>=half）
        if k >= half:
            if same_depot:
                for mask in feasible_masks_k[0]:
                    comp = FULL ^ mask
                    solve_vehicle(0, comp)
                    if vehicle_feasible[0].get(mask, False) and vehicle_feasible[0].get(comp, False):
                        total = vehicle_cost[0][mask] + vehicle_cost[0][comp]
                        if total < best_total:
                            best_total = total
                            best_r0_idx = vehicle_route[0][mask]
                            best_r1_idx = vehicle_route[0][comp]
            else:
                for v in (0, 1):
                    other = 1 - v
                    for mask in feasible_masks_k[v]:
                        comp = FULL ^ mask
                        solve_vehicle(other, comp)
                        if vehicle_feasible[v].get(mask, False) and vehicle_feasible[other].get(comp, False):
                            total = vehicle_cost[v][mask] + vehicle_cost[other][comp]
                            if total < best_total:
                                best_total = total
                                best_r0_idx = vehicle_route[v][mask]
                                best_r1_idx = vehicle_route[other][comp]

    if best_r0_idx is None or best_r1_idx is None:
        raise RuntimeError("2車両の厳密解が見つかりません（入力が不可能制約の可能性）。")

    # =========================================================
    # VERIFY BLOCK (optional)  -- works for same_depot True/False
    # =========================================================
    if VERIFY:

        def _solve_mask_for_vehicle(v: int, mask: int) -> Optional[List[int]]:
            """
            検証用：現在の増分solverをそのまま使って mask を解く。
            （別実装のDP等で独立検証したい場合は、ここを差し替える）
            """
            solve_vehicle(v, mask)
            return vehicle_route[v].get(mask)

        # ---- (A) vehicle-feasible の完全検証 ----
        if VERIFY_VEH_FEAS:
            solver_feasible: List[Dict[int, bool]] = [{}, {}]
            for v in (0, 1):
                for mask in range(FULL + 1):
                    solver_feasible[v][mask] = (_solve_mask_for_vehicle(v, mask) is not None)

            mismatches = []
            for v in (0, 1):
                for mask in range(FULL + 1):
                    cache_val = bool(vehicle_feasible[v].get(mask, False))
                    sol_val = bool(solver_feasible[v][mask])
                    if cache_val != sol_val:
                        mismatches.append((v, mask, cache_val, sol_val))

            if mismatches:
                print(f"[VERIFY] vehicle_feasible mismatches = {len(mismatches)}")
                for (v, mask, cache_val, sol_val) in mismatches[:50]:
                    pairs_id = mask_to_pairs_id(mask)
                    print("--------------------------------------------------")
                    print(f"vehicle={v} mask={mask} bin={bin(mask)} k={mask.bit_count()}")
                    print(f"pairs(ID)={pairs_id}")
                    print(f"vehicle_feasible(cache)={cache_val} solver={sol_val}")
                    if mask in subset_pruned_bad[v]:
                        bad = subset_pruned_bad[v][mask]
                        print(
                            f"subset_pruned_bad={bad} bin(bad)={bin(bad)} "
                            f"bad_pairs(ID)={mask_to_pairs_id(bad)}"
                        )
                raise RuntimeError(
                    f"[VERIFY FAILED] vehicle_feasible と solver の判定が一致しませんでした: mismatch={len(mismatches)}"
                )

            print("[VERIFY] vehicle_feasible matches solver for ALL masks (both vehicles). OK.")

        # ---- (B) 2車両の真の最適（総当たり） ----
        if VERIFY_TRUE_OPT:
            solver_feasible: List[Dict[int, bool]] = [{}, {}]
            for v in (0, 1):
                for mask in range(FULL + 1):
                    solver_feasible[v][mask] = (_solve_mask_for_vehicle(v, mask) is not None)

            best_total_true = float("inf")
            best_pair_true: Optional[Tuple[List[int], List[int]]] = None

            # 向き固定：車両0が mask、車両1が comp（mask全列挙で逆向きも必ず評価される）
            for mask in range(FULL + 1):
                comp = FULL ^ mask
                if not solver_feasible[0][mask]:
                    continue
                if not solver_feasible[1][comp]:
                    continue

                r0 = vehicle_route[0][mask]
                r1 = vehicle_route[1][comp]
                c0 = vehicle_cost[0][mask]
                c1 = vehicle_cost[1][comp]
                total = c0 + c1
                if total < best_total_true:
                    best_total_true = total
                    best_pair_true = (r0, r1)

            if best_pair_true is None:
                raise RuntimeError(
                    "[VERIFY TRUE-OPT] 全割当総当たりでも feasible な2車両解が見つかりません。入力か solver に問題があります。"
                )

            if best_total != best_total_true:
                r0_true, r1_true = best_pair_true
                r0_true_id = [idx2id[i] for i in r0_true]
                r1_true_id = [idx2id[i] for i in r1_true]
                print("[VERIFY TRUE-OPT] best_total mismatch!")
                print(f"  strategy best_total = {best_total}")
                print(f"  true best_total     = {best_total_true}")
                print(f"  true r0(ID)={r0_true_id}")
                print(f"  true r1(ID)={r1_true_id}")
                raise RuntimeError(
                    "[VERIFY TRUE-OPT FAILED] 本戦略のbest_totalが真の最適と一致しません。"
                )

            print("[VERIFY TRUE-OPT] strategy best_total matches true optimum. OK.")

    # -----------------------------
    # 8) index → IDに直して出力
    # -----------------------------
    best_r0_id = [idx2id[i] for i in best_r0_idx]
    best_r1_id = [idx2id[i] for i in best_r1_idx]
    return [best_r0_id, best_r1_id]



def debug_run_exact_2vehicle_vrp_on_two_routes(
    all_customers: List[Dict],
    all_PD_pairs: Dict[int, int],  # {pickup_id: delivery_id}
    vehicle_capacity: int = 200,
) -> List[List[int]]:
    # あなたが指定した2本のルート（ID列）
    r0 = [0, 113, 155, 78, 175, 13, 43, 204, 2, 90, 67, 39, 107, 0]
    r1 =  [0, 325, 370, 249, 361, 233, 362, 227, 351, 414, 398, 353, 322, 274, 267, 0]

    # ルートに登場するID集合（デポ含む）
    node_ids = set(r0) | set(r1)

    # customers 抽出
    id2cust_all = {c["id"]: c for c in all_customers}
    missing = [nid for nid in sorted(node_ids) if nid not in id2cust_all]
    if missing:
        raise ValueError(f"all_customers に存在しない node_id があります: {missing}")

    sub_customers = [id2cust_all[nid] for nid in sorted(node_ids)]

    # PD抽出：pickup,delivery の両方が node_ids にあるものだけ
    sub_PD_pairs: List[Tuple[int, int]] = []
    for p, d in all_PD_pairs.items():
        if p in node_ids and d in node_ids:
            sub_PD_pairs.append((p, d))

    # デポ（両車両とも 0→0 を想定）
    start_depots = [0, 0]
    end_depots = [0, 0]

    print("=== DEBUG INPUT for solve_exact_2vehicle_vrp ===")
    print(f"route0={r0}")
    print(f"route1={r1}")
    print(f"node_ids(sorted)={sorted(node_ids)}")
    print(f"sub_customers len={len(sub_customers)}")
    print(f"sub_PD_pairs={sub_PD_pairs}")
    print(f"start_depots={start_depots}, end_depots={end_depots}, cap={vehicle_capacity}")
    print("===============================================")

    # 直接実行
    new_routes = solve_exact_2vehicle_vrp(
        sub_customers=sub_customers,
        sub_PD_pairs=sub_PD_pairs,   # list[(pickup_id, delivery_id)] すべてID
        start_depots=start_depots,   # ID
        end_depots=end_depots,       # ID
        vehicle_capacity=vehicle_capacity,
    )

    print("=== OUTPUT from solve_exact_2vehicle_vrp ===")
    print(new_routes)
    return new_routes