from typing import Dict, List, Tuple
import math
from flexible_vrp_solver import solve_vrp_flexible

def perform_voronoi_routing_onlyMovedPD(
    routes: List[List[int]],
    customers: List[Dict],
    PD_pairs: Dict[int, int],
    depot_id_list: List[int],
    vehicle_num_list: List[int],
    vehicle_capacity: int,
):
    """
    ボロノイ分割（最近デポ）で PD を再配布するが、
    実際に solve_vrp_flexible で再最適化するのは「割当先会社が変わったPDペアのみ」。

    - before_company: routes から推定（(p,d)が同一車両に含まれる会社）
    - after_company : 重心→最寄りデポで決定
    - 固定車両     : その車両が「不変PD」を1つでも含むなら固定（そのルートは保存）
      ただし固定ルートからは moved PD ノードだけ除去して保存
    - 使用可能車両台数 = 全車両台数 - 固定車両台数
    """

    # id→node辞書
    id_to_node = {c["id"]: c for c in customers}
    id_to_coord = {c["id"]: (float(c["x"]), float(c["y"])) for c in customers}

    # 会社ごとのルートに分割
    per_company_routes: List[List[List[int]]] = []
    idx = 0
    for nveh in vehicle_num_list:
        per_company_routes.append(routes[idx: idx + nveh])
        idx += nveh
    assert idx == len(routes), "routes の本数が vehicle_num_list と一致しません"

    # -------------------------
    # ユーティリティ
    # -------------------------
    def nearest_company_of_midpoint(p_id: int, d_id: int) -> int:
        px, py = id_to_coord[p_id]
        dx, dy = id_to_coord[d_id]
        cx, cy = (px + dx) / 2.0, (py + dy) / 2.0

        best_comp = 0
        best_dist = float("inf")
        for comp_idx, depot_id in enumerate(depot_id_list):
            depot_x, depot_y = id_to_coord[depot_id]
            dist = math.hypot(cx - depot_x, cy - depot_y)
            if dist < best_dist:
                best_dist = dist
                best_comp = comp_idx
        return best_comp

    def vehicle_contains_pd(route: List[int], p_id: int, d_id: int) -> bool:
        s = set(route)
        return (p_id in s) and (d_id in s)

    def remove_nodes_keep_depots(route: List[int], remove_set: set) -> List[int]:
        if not route:
            return route
        if len(route) <= 2:
            return route
        start = route[0]
        end = route[-1]
        middle = [n for n in route[1:-1] if n not in remove_set]
        return [start] + middle + [end]

    # -------------------------
    # before_company / after_company を計算
    # -------------------------
    before_company_of: Dict[Tuple[int, int], int] = {}
    after_company_of: Dict[Tuple[int, int], int] = {}
    moved_pairs: List[Tuple[int, int]] = []
    unchanged_pairs: List[Tuple[int, int]] = []
    moved_nodes = set()
    
    company_id_ranges = []
    max_customer_id = max(c["id"] for c in customers)
    for comp_idx, dep_id in enumerate(depot_id_list):
        next_dep = depot_id_list[comp_idx + 1] if comp_idx + 1 < len(depot_id_list) else (max_customer_id + 1)
        lo = dep_id
        hi = next_dep - 1
        company_id_ranges.append((lo, hi))
    
    def company_of_node_by_id(node_id: int) -> int:
        for comp_idx, (lo, hi) in enumerate(company_id_ranges):
            if lo <= node_id <= hi:
                return comp_idx
        raise AssertionError(f"node_id={node_id} がどの会社ID範囲にも入りません。depot_id_list/ID設計を確認してください。")

    
    for p_id, d_id in PD_pairs.items():
        # before_company（ID範囲で決定）
        before_company_of[(p_id, d_id)] = company_of_node_by_id(p_id)
         # after_company（重心→最寄りデポ）
        after_company_of[(p_id, d_id)] = nearest_company_of_midpoint(p_id, d_id)
        
        if before_company_of[(p_id, d_id)] != after_company_of[(p_id, d_id)]:
            moved_pairs.append((p_id, d_id))
            moved_nodes.add(p_id)
            moved_nodes.add(d_id)
        else:
            unchanged_pairs.append((p_id, d_id))

    # -------------------------
    # 固定車両の判定と「保存する固定ルート」の作成
    # -------------------------
    unchanged_set = set(unchanged_pairs)

    fixed_routes_per_company: List[List[List[int]]] = [[] for _ in depot_id_list]
    fixed_vehicle_count: List[int] = [0] * len(depot_id_list)

    for comp_idx, comp_routes in enumerate(per_company_routes):
        for route in comp_routes:
            # その車両が不変PDを1つでも含むなら固定車両
            is_fixed = False
            for p_id, d_id in unchanged_set:
                if vehicle_contains_pd(route, p_id, d_id):
                    is_fixed = True
                    break

            if is_fixed:
                fixed_vehicle_count[comp_idx] += 1
                # 固定経路から moved PD ノードだけ除去して保存
                fixed_route = remove_nodes_keep_depots(route, moved_nodes)
                fixed_routes_per_company[comp_idx].append(fixed_route)

    # -------------------------
    # 各社ごと：moved PD だけでVRPを解く
    # -------------------------
    all_routes_out: List[List[int]] = []

    for comp_idx, depot_id in enumerate(depot_id_list):
        total_veh = vehicle_num_list[comp_idx]
        fixed_veh = fixed_vehicle_count[comp_idx]
        free_veh = total_veh - fixed_veh

        # この会社に「after割当」された moved PD のみ
        moved_pairs_for_company = [
            (p, d) for (p, d) in moved_pairs if after_company_of[(p, d)] == comp_idx
        ]

        # まず固定ルートを入れる
        preserved = fixed_routes_per_company[comp_idx]
        # 念のため台数オーバー防止
        if len(preserved) > total_veh:
            preserved = preserved[:total_veh]
        all_routes_out.extend(preserved)

        # free車両が無い or movedが無いなら、残りは空ルートで埋める
        if free_veh <= 0 or len(moved_pairs_for_company) == 0:
            rest = total_veh - len(preserved)
            all_routes_out.extend([[depot_id, depot_id] for _ in range(rest)])
            continue

        # customers（デポ + moved ノードだけ）
        moved_node_ids = {depot_id}
        for p, d in moved_pairs_for_company:
            moved_node_ids.add(p)
            moved_node_ids.add(d)
        sub_customers = [id_to_node[nid] for nid in moved_node_ids]

        starts = [depot_id] * free_veh
        ends = [depot_id] * free_veh

        print(
            f">>> Voronoi(変更PDのみ) LSP {comp_idx+1}: "
            f"fixed={fixed_veh}台, free={free_veh}台, movedPD={len(moved_pairs_for_company)}組"
        )

        new_routes = solve_vrp_flexible(
            customers=sub_customers,
            initial_routes=None,
            PD_pairs=moved_pairs_for_company,
            num_vehicles=free_veh,
            vehicle_capacity=vehicle_capacity,
            start_depots=starts,
            end_depots=ends,
            use_capacity=True,
            use_time=True,
            use_pickup_delivery=True,
        )

        if new_routes is None:
            print(f"⚠️ LSP {comp_idx+1}: 解が見つからなかったため free分は空ルートを採用")
            new_routes = [[depot_id, depot_id] for _ in range(free_veh)]

        # 台数がズレたら合わせる
        if len(new_routes) < free_veh:
            new_routes = new_routes + [[depot_id, depot_id] for _ in range(free_veh - len(new_routes))]
        elif len(new_routes) > free_veh:
            new_routes = new_routes[:free_veh]

        all_routes_out.extend(new_routes)

    return all_routes_out
