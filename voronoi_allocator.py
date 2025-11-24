from typing import Dict, List, Tuple
import math
from flexible_vrp_solver import solve_vrp_flexible

def perform_voronoi_routing(
    customers: List[Dict],
    PD_pairs: Dict[int, int],
    depot_id_list: List[int],
    vehicle_num_list: List[int],
    vehicle_capacity: int,
):
    """
    ボロノイ分割（最近デポ）でタスクを各社に再配布し、その後 各社独立にVRPを一発最適化して
    全車両ルート（デポ始終の巡回リスト）を返す。

    ルール：
      - PDペアは pickup+delivery を同一会社に配属。
      - 会社の決定は「PDペアの重心（中点）からデポまでの距離」が最小の会社。
      - デポノード（demand==0）は各社の sub_customers に必ず含める。
      - PDに属さないノードが存在する場合は、当該ノードから最も近いデポの会社に配属。
    """
    # ID→ノード辞書 & 座標
    id_to_node = {c["id"]: c for c in customers}
    id_to_coord = {c["id"]: (float(c["x"]), float(c["y"])) for c in customers}

    # 会社ごとのコンテナ
    company_customers: List[List[Dict]] = [[] for _ in depot_id_list]
    company_pd_pairs: List[List[Tuple[int, int]]] = [[] for _ in depot_id_list]

    # 各社のデポを先に sub_customers に入れておく
    for comp_idx, depot_id in enumerate(depot_id_list):
        company_customers[comp_idx].append(id_to_node[depot_id])

    # --- PDペアの割当：重心ベース ---
    for p_id, d_id in PD_pairs.items():
        if p_id not in id_to_coord or d_id not in id_to_coord:
            print(f"⚠️ Invalid PD pair: ({p_id}, {d_id})")
            continue

        px, py = id_to_coord[p_id]
        dx, dy = id_to_coord[d_id]
        # 中点（重心）
        cx, cy = (px + dx) / 2.0, (py + dy) / 2.0

        # 重心から最も近いデポを持つ会社を選ぶ
        best_comp = None
        best_dist = float("inf")
        for comp_idx, depot_id in enumerate(depot_id_list):
            depot_x, depot_y = id_to_coord[depot_id]
            dist = math.hypot(cx - depot_x, cy - depot_y)
            if dist < best_dist:
                best_dist = dist
                best_comp = comp_idx

        # その会社に pickup / delivery を配属（重複追加は避ける）
        for nid in (p_id, d_id):
            node = id_to_node[nid]
            if all(node["id"] != c["id"] for c in company_customers[best_comp]):
                company_customers[best_comp].append(node)

        company_pd_pairs[best_comp].append((p_id, d_id))

    # 非PDノード（デポ以外）が混ざっていないことを厳密にチェック
    pd_nodes = set(list(PD_pairs.keys()) + list(PD_pairs.values()))
    extra = [
        c for c in customers
        if c["id"] not in pd_nodes and c["id"] not in depot_id_list
    ]
    if extra:
        # 厳密運用：想定外のノードがあるなら即停止して気付けるようにする
        ids = [c["id"] for c in extra]
        raise ValueError(
            f"Non-PD (non-depot) nodes detected: {len(extra)} nodes. IDs={ids[:10]}..."
        )


    # --- 各社で独立にVRPを解き、ルートを連結 ---
    all_routes: List[List[int]] = []
    for comp_idx, depot_id in enumerate(depot_id_list):
        sub_customers = company_customers[comp_idx]
        sub_pd_pairs = company_pd_pairs[comp_idx]
        num_vehicles = vehicle_num_list[comp_idx]

        starts = [depot_id] * num_vehicles
        ends = [depot_id] * num_vehicles

        print(
            f">>> Voronoi分割中 LSP {comp_idx+1}: "
            f"顧客={len(sub_customers)}件, 車両={num_vehicles}台, PD={len(sub_pd_pairs)}組"
        )

        routes = solve_vrp_flexible(
            customers=sub_customers,
            initial_routes=None,
            PD_pairs=sub_pd_pairs,
            num_vehicles=num_vehicles,
            vehicle_capacity=vehicle_capacity,
            start_depots=starts,
            end_depots=ends,
            use_capacity=True,
            use_time=True,
            use_pickup_delivery=True,
            InitialRoute=False
        )
        if routes is None:
            print(f"⚠️ LSP {comp_idx+1}: 解が見つからなかったため空ルートを採用")
            routes = [[depot_id, depot_id] for _ in range(num_vehicles)]

        all_routes.extend(routes)

    return all_routes



def perform_voronoi_routing_with_initRoute(
    customers: List[Dict],
    PD_pairs: Dict[int, int],
    depot_id_list: List[int],
    vehicle_num_list: List[int],
    vehicle_capacity: int,
    original_routes: List[List[int]],   # ★ ステップ1で作った全車両の初期経路（デポ付き）
):
    """
    ボロノイ分割（最近デポ）でタスクを各社に再配布し、その後 各社独立にVRPを一発最適化して
    全車両ルート（デポ始終の巡回リスト）を返す。

    ルール：
      - PDペアは pickup+delivery を同一会社に配属。
      - 会社の決定は「PDペアの重心（中点）からデポまでの距離」が最小の会社。
      - デポノード（demand==0）は各社の sub_customers に必ず含める。
      - PDに属さないノードが存在する場合は、当該ノードから最も近いデポの会社に配属。

    さらに：
      - original_routes（ステップ1の初期経路）をもとに、ボロノイ再配分後の
        各社・各車両の initial_routes を人為的に構成し、OR-Tools の
        ReadAssignmentFromRoutes 相当のウォームスタートに利用する。
    """
    # ID→ノード辞書 & 座標
    id_to_node = {c["id"]: c for c in customers}
    id_to_coord = {c["id"]: (float(c["x"]), float(c["y"])) for c in customers}

    # 会社ごとのコンテナ
    company_customers: List[List[Dict]] = [[] for _ in depot_id_list]
    company_pd_pairs: List[List[Tuple[int, int]]] = [[] for _ in depot_id_list]

    # 各社のデポを先に sub_customers に入れておく
    for comp_idx, depot_id in enumerate(depot_id_list):
        company_customers[comp_idx].append(id_to_node[depot_id])

    # --- PDペアの割当：重心ベース ---
    for p_id, d_id in PD_pairs.items():
        if p_id not in id_to_coord or d_id not in id_to_coord:
            print(f"⚠️ Invalid PD pair: ({p_id}, {d_id})")
            continue

        px, py = id_to_coord[p_id]
        dx, dy = id_to_coord[d_id]
        # 中点（重心）
        cx, cy = (px + dx) / 2.0, (py + dy) / 2.0

        # 重心から最も近いデポを持つ会社を選ぶ
        best_comp = None
        best_dist = float("inf")
        for comp_idx, depot_id in enumerate(depot_id_list):
            depot_x, depot_y = id_to_coord[depot_id]
            dist = math.hypot(cx - depot_x, cy - depot_y)
            if dist < best_dist:
                best_dist = dist
                best_comp = comp_idx

        # その会社に pickup / delivery を配属（重複追加は避ける）
        for nid in (p_id, d_id):
            node = id_to_node[nid]
            if all(node["id"] != c["id"] for c in company_customers[best_comp]):
                company_customers[best_comp].append(node)

        company_pd_pairs[best_comp].append((p_id, d_id))

    # 非PDノード（デポ以外）が混ざっていないことを厳密にチェック
    pd_nodes = set(list(PD_pairs.keys()) + list(PD_pairs.values()))
    extra = [
        c for c in customers
        if c["id"] not in pd_nodes and c["id"] not in depot_id_list
    ]
    if extra:
        # 厳密運用：想定外のノードがあるなら即停止して気付けるようにする
        ids = [c["id"] for c in extra]
        raise ValueError(
            f"Non-PD (non-depot) nodes detected: {len(extra)} nodes. IDs={ids[:10]}..."
        )

    # ======================================================================
    # ★ ここから：original_routes を使った initial_routes の構築
    # ======================================================================

    # 会社ごとの「担当ノードID集合」（デポ含む）を作る
    company_assigned_ids: List[set] = [
        {c["id"] for c in comp_customers} for comp_customers in company_customers
    ]

    # original_routes を会社ごとに分割
    original_routes_per_company: List[List[List[int]]] = []
    idx = 0
    for n in vehicle_num_list:
        original_routes_per_company.append(original_routes[idx: idx + n])
        idx += n

    # まずは「自社担当ノードだけ残したルート」を initial_routes のたたき台にする
    # （デポは削除、PD もボロノイで他社担当ならここで落ちる）
    initial_routes_per_company: List[List[List[int]]] = []
    for comp_idx, routes_list in enumerate(original_routes_per_company):
        assigned_ids = company_assigned_ids[comp_idx]
        depot_id = depot_id_list[comp_idx]

        comp_init_routes: List[List[int]] = []
        for r in routes_list:
            # デポ始終の巡回 [D, ..., D] 前提で、内部ノードだけを取り出す
            if len(r) >= 2 and r[0] == r[-1] == depot_id:
                core = r[1:-1]
            else:
                core = r[:]

            # この会社に属さないノードは削除（デポも initial_routes には入れない）
            filtered = [
                nid for nid in core
                if (nid in assigned_ids) and (nid not in depot_id_list)
            ]
            comp_init_routes.append(filtered)

        initial_routes_per_company.append(comp_init_routes)

    # さらに PD 単位でチェックし、担当会社の initial_routes に必ず [pickup, delivery] が
    # どこかのルートに両方とも含まれるように強制する
    for comp_idx, pd_list in enumerate(company_pd_pairs):
        routes_list = initial_routes_per_company[comp_idx]
        if not routes_list:
            continue

        for (p_id, d_id) in pd_list:
            # 既にどこかのルートに含まれているか？
            target_rt = None
            for rt in routes_list:
                if p_id in rt or d_id in rt:
                    target_rt = rt
                    break

            if target_rt is None:
                # どのルートにも居ない → 空ルートを探す
                for rt in routes_list:
                    if len(rt) == 0:
                        target_rt = rt
                        break
                # 空ルートが無ければ先頭ルートを使う
                if target_rt is None:
                    target_rt = routes_list[0]

            # target_rt に pickup, delivery を (無ければ) 追加
            if p_id not in target_rt:
                target_rt.append(p_id)
            if d_id not in target_rt:
                target_rt.append(d_id)

    # ======================================================================
    # === 各社で独立にVRPを解き、ルートを連結（initial_routes を利用） ===
    # ======================================================================
    all_routes: List[List[int]] = []
    for comp_idx, depot_id in enumerate(depot_id_list):
        sub_customers = company_customers[comp_idx]
        sub_pd_pairs = company_pd_pairs[comp_idx]
        num_vehicles = vehicle_num_list[comp_idx]

        starts = [depot_id] * num_vehicles
        ends = [depot_id] * num_vehicles

        print(
            f">>> Voronoi分割中 LSP {comp_idx+1}: "
            f"顧客={len(sub_customers)}件, 車両={num_vehicles}台, PD={len(sub_pd_pairs)}組"
        )

        # この会社用に作った initial_routes（デポ除去済みの顧客ID列）
        init_routes_for_company = initial_routes_per_company[comp_idx]

        routes = solve_vrp_flexible(
            customers=sub_customers,
            initial_routes=init_routes_for_company,   # ★ ウォームスタートとして渡す
            PD_pairs=sub_pd_pairs,                    # List[Tuple[int,int]]
            num_vehicles=num_vehicles,
            vehicle_capacity=vehicle_capacity,
            start_depots=starts,
            end_depots=ends,
            use_capacity=True,
            use_time=True,
            use_pickup_delivery=True,
            InitialRoute=True                         # ★ 初期ルートを使うフラグ
        )
        if routes is None:
            print(f"⚠️ LSP {comp_idx+1}: 解が見つからなかったため空ルートを採用")
            routes = [[depot_id, depot_id] for _ in range(num_vehicles)]

        all_routes.extend(routes)

    return all_routes
