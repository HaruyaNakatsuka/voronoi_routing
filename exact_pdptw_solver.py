from __future__ import annotations
from typing import Dict, List, Tuple, Optional, TypeAlias
from collections import defaultdict
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
# solve_1vehicle_pdptw_exact_idxの探索フェーズで枝刈りを行うための
# 上界を簡易的(貪欲)に生成するための関数
# 解が得られない場合もある
# ============================================================
def build_any_feasible_route_greedy_idx(
    visit_idxs: List[int],
    idx_to_bit: dict[int, int],
    delivery_to_pickup: dict[int, int],
    demand: List[int],
    ready: List[int],
    due: List[int],
    service: List[int],
    dist_mat: List[List[float]],
    start_depot_idx: int,
    end_depot_idx: int,
    vehicle_capacity: int,
) -> Optional[Tuple[List[int], float]]:
    """
    1台分の feasible ルートを貪欲に1本作る。
    返り値: (route_idxs, total_dist) or None
    """
    # 状態
    mask = 0
    last = start_depot_idx
    load = 0
    t = max(0.0, float(ready[start_depot_idx]))
    if t > due[start_depot_idx]:
        return None

    route = [start_depot_idx]
    full_mask = (1 << len(visit_idxs)) - 1
    total_dist = 0.0

    # precedence check（deliveryはpickup済みが必要）
    def precedence_ok(m: int, nxt: int) -> bool:
        if nxt in delivery_to_pickup:
            p = delivery_to_pickup[nxt]
            pb = idx_to_bit[p]  # ※ visit_idxs 由来なので p は必ず入っている前提
            return ((m >> pb) & 1) == 1
        return True

    while mask != full_mask:
        candidates = []
        for nxt in visit_idxs:
            b = idx_to_bit[nxt]
            if (mask >> b) & 1:
                continue
            if not precedence_ok(mask, nxt):
                continue

            new_load = load + demand[nxt]
            if new_load < 0 or new_load > vehicle_capacity:
                continue

            travel = dist_mat[last][nxt]
            arrive = t + float(service[last]) + travel
            new_t = max(arrive, float(ready[nxt]))
            if new_t > due[nxt]:
                continue

            # スコア：近さ + 早い締切（好きに変えてOK）
            # 例）「行って戻るまでの見込み」+「締切を少し意識」
            score = travel + dist_mat[nxt][end_depot_idx] + 0.001 * float(due[nxt])
            candidates.append((score, nxt, new_load, new_t, travel))

        if not candidates:
            return None  # 貪欲では詰んだ

        candidates.sort(key=lambda x: x[0])
        _, nxt, new_load, new_t, travel = candidates[0]

        # 遷移確定
        b = idx_to_bit[nxt]
        mask |= (1 << b)
        route.append(nxt)
        total_dist += travel
        last = nxt
        load = new_load
        t = new_t

    # endへ戻れるかチェック
    travel_back = dist_mat[last][end_depot_idx]
    arrive_end = t + float(service[last]) + travel_back
    end_t = max(arrive_end, float(ready[end_depot_idx]))
    if end_t > due[end_depot_idx]:
        return None

    route.append(end_depot_idx)
    total_dist += travel_back
    return route, total_dist


# ============================================================
# 1台の PDPTW を「厳密に」解く（ラベリング/DP）
# - 目的: 距離最小
# - 制約: 容量 / タイムウィンドウ / PD順序（pickup -> delivery）
# - 入力は Index 基準
# ============================================================
def solve_1vehicle_pdptw_exact_idx(
    sub_customers: List[Dict],
    pd_pairs: List[Tuple[int, int]],      # indexペア (pickup_idx, delivery_idx)
    start_depot_idx: int,                 # index
    end_depot_idx: int,                   # index
    vehicle_capacity: int,
    dist_mat: List[List[float]],
    move_time: List[List[float]],
) -> Optional[List[int]]:
    """
    1車両の PDPTW を厳密に解いて、ルート（index列）を返す。
    返り値: [start_idx, ..., end_idx]（存在しなければ None）

    目的:
      - 距離最小（ユークリッド距離 dist_mat）
    時間:
      - 到着時刻更新:
          arrive = time_sofar + move_time[last][nxt]
          new_time = max(arrive, ready[nxt])
        ※ move_time[i][j] = service[i] + dist_mat[i][j] を想定
    PD順序:
      - ペア単位の3値状態（0=未着手, 1=pickup済, 2=delivery済）で管理する。
    """

    # ========== 前処理（配列で持つ：index基準） ==========
    demand  = [int(c["demand"])  for c in sub_customers]
    ready   = [int(c["ready"])   for c in sub_customers]
    due     = [int(c["due"])     for c in sub_customers]
    service = [int(c["service"]) for c in sub_customers]

    # ========== PDペア列の整理 ==========
    # pd_pairs が空ならデポ直行
    if not pd_pairs:
        t0 = max(0.0, float(ready[start_depot_idx]))
        if t0 > float(due[start_depot_idx]):
            return None

        travel = dist_mat[start_depot_idx][end_depot_idx]
        arrive_end = t0 + float(service[start_depot_idx]) + travel
        t_end = max(arrive_end, float(ready[end_depot_idx]))
        if t_end > float(due[end_depot_idx]):
            return None

        return [start_depot_idx, end_depot_idx]

    # ここからは「ペア数 m」を基準に 3^m の状態を使う
    pairs: List[Tuple[int, int]] = list(pd_pairs)
    m = len(pairs)

    # ========== 3進マスク（状態 0/1/2）準備 ==========
    pow3 = [1] * (m + 1)
    for i in range(m):
        pow3[i + 1] = pow3[i] * 3

    # 全ペアが 2（delivery済）である状態がゴール
    full_mask = 0
    for i in range(m):
        full_mask += 2 * pow3[i]

    # 3進マスクから「ペアiの状態」を取り出す
    def get_state(mask: int, i: int) -> int:
        return (mask // pow3[i]) % 3  # 0,1,2

    # 3進マスクで「ペアiの状態を +1」進める（0->1 or 1->2）
    def inc_state(mask: int, i: int) -> int:
        return mask + pow3[i]

    # ========== ラベルDP（Pareto刈り） ==========
    # 状態(state) = (mask3, last, load)
    #   - mask3 : 各PDペアの進捗（0/1/2）を3進で表す
    #   - last  : 現在いるノード(index)
    #   - load  : 現在積載量
    #
    # ラベル(label) = (dist, time, prev)
    #   - dist : ここまでの総距離
    #   - time : last 到着時刻（TW判定に使う）
    #   - prev : (pmask, plast, pload, last_added)  復元用
    Label: TypeAlias = Tuple[float, float, Optional[Tuple[int, int, int, int]]]

    table: Dict[Tuple[int, int, int], List[Label]] = defaultdict(list)
    mask_to_keys: Dict[int, List[Tuple[int, int, int]]] = defaultdict(list)
    key_seen: set[Tuple[int, int, int]] = set()

    # 初期状態：デポ開始、積載0
    t0 = max(0.0, float(ready[start_depot_idx]))
    if t0 > float(due[start_depot_idx]):
        return None

    init_key = (0, start_depot_idx, 0)
    table[init_key].append((0.0, t0, None))
    mask_to_keys[0].append(init_key)
    key_seen.add(init_key)

    def insert_label(key: Tuple[int, int, int], cand: Label) -> None:
        """同一key内で (dist,time) の支配関係により刈りつつ挿入"""
        dist_c, time_c, _ = cand
        lst = table[key]

        # 既存が cand を支配 → 捨て
        for dist_e, time_e, _ in lst:
            if dist_e <= dist_c and time_e <= time_c:
                return

        # cand が既存を支配 → 既存を除去
        new_lst: List[Label] = []
        for dist_e, time_e, prev_e in lst:
            if not (dist_c <= dist_e and time_c <= time_e):
                new_lst.append((dist_e, time_e, prev_e))
        new_lst.append(cand)
        table[key] = new_lst

        if key not in key_seen:
            key_seen.add(key)
            mask_to_keys[key[0]].append(key)

    # ========== 枝刈り用の上界（既存の貪欲を流用） ==========
    # 既存の build_any_feasible_route_greedy_idx が「ノード集合 visit_idxs」を要求するなら、
    # ここでは「全pickup+deliveryを訪問対象」として与える。
    best_total = float("inf")
    visit_set = set()
    for p, d in pairs:
        visit_set.add(p)
        visit_set.add(d)
    visit_idxs = sorted(visit_set)

    try:
        greedy = build_any_feasible_route_greedy_idx(
            visit_idxs=visit_idxs,
            idx_to_bit={nid: i for i, nid in enumerate(visit_idxs)},
            delivery_to_pickup={d: p for p, d in pairs},
            demand=demand,
            ready=ready,
            due=due,
            service=service,
            dist_mat=dist_mat,
            start_depot_idx=start_depot_idx,
            end_depot_idx=end_depot_idx,
            vehicle_capacity=vehicle_capacity,
        )
        if greedy is not None:
            _route_g, ub = greedy
            best_total = ub
    except NameError:
        # build_any_feasible_route_greedy_idx が未定義でも、DP自体は動く（ただし枝刈り弱くなる）
        pass

    # ========== 遷移（3^m） ==========
    # mask を 0..full_mask で回すのではなく、到達済み mask のみ走査するために
    # mask_to_keys を使う（ただし full_mask は "値" が飛ぶので range では回さない）。
    # ここは「mask_to_keys のキー集合を昇順で走査」する方式にする。
    # ただし insert_label により新maskが増えるので、動的に処理するため while を使う。
    pending_masks = [0]
    pending_seen = {0}
    ptr = 0

    while ptr < len(pending_masks):
        mask = pending_masks[ptr]
        ptr += 1

        keys = mask_to_keys.get(mask, [])
        for (m3, last, load) in keys:
            for dist_sofar, time_sofar, _prev in list(table[(m3, last, load)]):

                # 枝刈り1：最低でも end に戻る必要がある
                if dist_sofar + dist_mat[last][end_depot_idx] >= best_total:
                    continue

                # 次に動かせるペアを列挙：
                # state=0 -> pickupへ
                # state=1 -> deliveryへ
                for i, (p, d) in enumerate(pairs):
                    st = get_state(m3, i)
                    if st == 2:
                        continue  # このペアは完了

                    nxt = p if st == 0 else d

                    # 容量
                    new_load = load + demand[nxt]
                    if new_load < 0 or new_load > vehicle_capacity:
                        continue

                    # 時刻
                    travel = dist_mat[last][nxt]
                    arrive = time_sofar + move_time[last][nxt]  # service[last] + dist(last,nxt)
                    new_time = max(arrive, float(ready[nxt]))
                    if new_time > float(due[nxt]):
                        continue

                    new_dist = dist_sofar + travel
                    new_mask = inc_state(m3, i)

                    # 枝刈り2：nxtに行っても最低でも end に戻る必要がある
                    if new_dist + dist_mat[nxt][end_depot_idx] >= best_total:
                        continue

                    new_key = (new_mask, nxt, new_load)
                    insert_label(
                        new_key,
                        (new_dist, new_time, (m3, last, load, nxt)),
                    )

                    # 新しいmaskが初登場ならキューに入れる
                    if new_mask not in pending_seen:
                        pending_seen.add(new_mask)
                        pending_masks.append(new_mask)

                    # 枝刈り3：全ペア完了に到達した瞬間、end に戻れるなら上界更新
                    if new_mask == full_mask:
                        back = dist_mat[nxt][end_depot_idx]
                        arrive_end = new_time + float(service[nxt]) + back
                        t_end = max(arrive_end, float(ready[end_depot_idx]))
                        if t_end <= float(due[end_depot_idx]):
                            total = new_dist + back
                            if total < best_total:
                                best_total = total

    # ========== 終了（endに戻れるものの中で最良を選ぶ） ==========
    best_total_final: Optional[float] = None
    best_state = None  # (mask,last,load)
    best_label = None  # (dist,time,prev)

    full_keys = mask_to_keys.get(full_mask, [])
    for (m3, last, load) in full_keys:
        for dist_sofar, time_sofar, prev in table[(m3, last, load)]:
            travel_back = dist_mat[last][end_depot_idx]
            arrive_end = time_sofar + float(service[last]) + travel_back
            end_time = max(arrive_end, float(ready[end_depot_idx]))
            if end_time > float(due[end_depot_idx]):
                continue

            total_dist = dist_sofar + travel_back
            if best_total_final is None or total_dist < best_total_final:
                best_total_final = total_dist
                best_state = (m3, last, load)
                best_label = (dist_sofar, time_sofar, prev)

    if best_state is None or best_label is None:
        return None

    # ========== 経路復元 ==========
    route_rev = [end_depot_idx]
    m3, last, load = best_state
    _, _, prev = best_label

    route_rev.append(last)

    # prev = (pmask, plast, pload, last_added) を辿る
    # ※ ここは「prevが一意に辿れる」前提（insert_labelの設計通り）
    while prev is not None:
        pm, plast, pload, _visited = prev
        route_rev.append(plast)

        # 次の prev を得るために、table[(pm, plast, pload)] のラベルから prev を取り出す。
        # ただし複数ラベルがあり得るので、prevを確実に辿れるように「どれでも良い」ではなく
        # “plast に到達したラベル”の prev を拾う必要がある。
        # 現状は簡易的に先頭を取る（既存実装と同じ方針）。
        next_prev = None
        for _d2, _t2, prev2 in table[(pm, plast, pload)]:
            next_prev = prev2
            break
        prev = next_prev

    route = list(reversed(route_rev))

    # 念のため start/end を整形
    if route[0] != start_depot_idx:
        route = [start_depot_idx] + [x for x in route if x not in (start_depot_idx, end_depot_idx)] + [end_depot_idx]
    if route[-1] != end_depot_idx:
        route = route[:-1] + [end_depot_idx]

    return route




def solve_exact_2vehicle_vrp(
    sub_customers: List[Dict],
    sub_PD_pairs: List[Tuple[int, int]],    # ID（indexではない）
    start_depots: List[int],   # ID（indexではない）
    end_depots: List[int],     # ID（indexではない）
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

    # -----------------------------
    # 0) バリデーション
    # -----------------------------
    #必須キーの確認
    required_keys = {"id", "x", "y", "demand", "ready", "due", "service"}
    for c in sub_customers:
        missing = required_keys - set(c.keys())
        if missing:
            raise ValueError(f"sub_customers に必要キーが不足: missing={missing}, customer={c}")

    # デポリストの長さ = 車両台数 = 2
    if len(start_depots) != 2 or len(end_depots) != 2:
        raise ValueError("start_depots/end_depots は長さ2（2車両）である必要があります。")

    # sub_customers: id の重複チェック
    ids = [c["id"] for c in sub_customers]
    if len(ids) != len(set(ids)):
        raise ValueError("sub_customers 内に 'id' の重複があります。")

    id2cust = {c["id"]: c for c in sub_customers}

    # デポIDが sub_customers に存在するか
    for dep_id in start_depots + end_depots:
        if dep_id not in id2cust:
            raise ValueError(f"デポID {dep_id} が sub_customers に存在しません。")

    # PDペアのIDが sub_customers に存在するか
    # （あなたの方針：起きない前提なら assert/例外で止める）
    for p_id, d_id in sub_PD_pairs:
        if p_id not in id2cust or d_id not in id2cust:
            raise ValueError(
                f"sub_PD_pairs に sub_customers 外のIDが含まれています: ({p_id},{d_id})"
            )

    # -----------------------------
    # 1) 内部計算用に index 化（ID→index）
    # -----------------------------
    # 厳密解探索（DP/分枝限定/ラベリング）では配列indexが便利なので内部で変換する。
    id2idx = {c["id"]: i for i, c in enumerate(sub_customers)}
    idx2id = [c["id"] for c in sub_customers]

    # start/end は "ID" で渡されるので、そのまま index に変換
    start_idx = [id2idx[dep_id] for dep_id in start_depots]
    end_idx   = [id2idx[dep_id] for dep_id in end_depots]

    # PDペアも index に変換して保持
    pd_idx_pairs: List[Tuple[int, int]] = [(id2idx[p], id2idx[d]) for p, d in sub_PD_pairs]

    # -----------------------------
    # 2) 2車両の厳密探索：PD割当
    # -----------------------------
    m = len(pd_idx_pairs)
    
    if m == 0:
        # PDが無いならデポ→デポのみ（ここでは厳密以前に確定）
        r0 = [start_depots[0], end_depots[0]]
        r1 = [start_depots[1], end_depots[1]]
        return [r0, r1]
    
    coords = [(float(c["x"]), float(c["y"])) for c in sub_customers]
    service = [float(c["service"]) for c in sub_customers]

    dist_mat = build_dist_mat(coords)
    move_time = build_move_time(service, dist_mat)

    # 最良解を保存しておくための変数
    best_total = float("inf")
    best_r0_idx: List[int] | None = None
    best_r1_idx: List[int] | None = None

    # 対称性破り:
    #    車両に区別がないので、(割当, 反転割当) は片方だけ探索すればよい。
    #    ここでは「先頭のPDを必ず車両0に割り当てる」ことで 2倍の重複探索を除去する。
    # mask bits は pd_idx_pairs[1:] に対応（長さ m-1）
    for mask in range(1 << (m - 1)):
        # 対称性破り:先頭のPDは車両0に割り当てる
        v0_pairs = [pd_idx_pairs[0]]
        v1_pairs: List[Tuple[int, int]] = []

        # このループにおける各PDの車両0,1への割り当てを確定
        for k in range(1, m):
            bit = (mask >> (k - 1)) & 1
            if bit == 0:
                v0_pairs.append(pd_idx_pairs[k])
            else:
                v1_pairs.append(pd_idx_pairs[k])

        # ここで各車両の部分問題（1車両PDPTW）を厳密に解く
        r0_idx = solve_1vehicle_pdptw_exact_idx(
            sub_customers=sub_customers,
            pd_pairs=v0_pairs,
            start_depot_idx=start_idx[0],
            end_depot_idx=end_idx[0],
            vehicle_capacity=vehicle_capacity,
            dist_mat=dist_mat,
            move_time=move_time,
        )
        if r0_idx is None:
            continue

        r1_idx = solve_1vehicle_pdptw_exact_idx(
            sub_customers=sub_customers,
            pd_pairs=v1_pairs,
            start_depot_idx=start_idx[1],
            end_depot_idx=end_idx[1],
            vehicle_capacity=vehicle_capacity,
            dist_mat=dist_mat,
            move_time=move_time,
        )
        if r1_idx is None:
            continue

        # 最良解の更新
        print("実行可能解を発見")
        c0 = route_cost_idx(r0_idx, dist_mat)
        c1 = route_cost_idx(r1_idx, dist_mat)
        total = c0 + c1
        if total < best_total:
            best_total = total
            best_r0_idx = r0_idx
            best_r1_idx = r1_idx

    if best_r0_idx is None or best_r1_idx is None:
        raise RuntimeError("2車両の厳密解が見つかりません（入力が不可能制約の可能性）。")

    # -----------------------------
    # 3) index → ID に戻して返す
    # -----------------------------
    best_r0_id = [idx2id[i] for i in best_r0_idx]
    best_r1_id = [idx2id[i] for i in best_r1_idx]
    return [best_r0_id, best_r1_id]