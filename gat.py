from flexible_vrp_solver import solve_vrp_flexible, route_cost
from ortools.sat.python import cp_model


def initialize_individual_vrps(customers, pickup_to_delivery, num_lsps, vehicle_num_list, depot_id_list, vehicle_capacity, seed=42):
    all_vehicle_routes = []
    
    for i in range(num_lsps):
        depot_id = depot_id_list[i]
        num_vehicles = vehicle_num_list[i]

        # sub_customersの抽出（デポを含む）
        if i < num_lsps - 1:
            id_min = depot_id_list[i]
            id_max = depot_id_list[i + 1]
            sub_customers = [c for c in customers if id_min <= c['id'] < id_max]
        else:
            id_min = depot_id_list[i]
            sub_customers = [c for c in customers if c['id'] >= id_min]
        
        # sub_PD_pairsの抽出
        sub_customer_ids = {c['id'] for c in sub_customers}
        sub_PD_pairs = [
            (pickup, delivery)
            for pickup, delivery in pickup_to_delivery.items()
            if pickup in sub_customer_ids or delivery in sub_customer_ids
        ]

        # 各車両の出発／終了デポ設定
        start_depot = [depot_id] * num_vehicles
        end_depot = [depot_id] * num_vehicles
        
        initial_routes = None
        # VRPを解く
        print(f">>>LSP {i+1}の初期経路を生成中・・・")
        lsp_routes = solve_vrp_flexible(
            sub_customers,
            initial_routes,
            sub_PD_pairs,
            num_vehicles=num_vehicles,
            vehicle_capacity=vehicle_capacity,
            start_depots=start_depot,
            end_depots=end_depot,
            use_capacity=True,
            use_time=True,
            use_pickup_delivery=True,
            isGAT=False
        )

        all_vehicle_routes.extend(lsp_routes)

    return all_vehicle_routes


def perform_gat_exchange(original_routes, customers, PD_pairs, vehicle_capacity, vehicle_num_list):
    """
    社内限定GAT：与えられた routes は単一会社ぶんのみを想定。
    - 2車両ペアごとに部分問題を解き、改善候補（アクション）を集める
    - 各車両は最大1回だけ変更されるようにCP-SATでアクションを選択
    - 会社間の個別合理性や会社マッピングは行わない（総距離改善のみ）
    """
    feasible_actions = []
    num_vehicles = len(original_routes)

    # 各車両ルートに関連する PD ペア（そのルートに現れるノードを含むペア）を前計算
    PD_pairs_of_each_vehicle = []
    for vehicle_route in original_routes:
        related_pairs = []
        visited_set = set(vehicle_route)
        for pickup, delivery in PD_pairs.items():
            if pickup in visited_set or delivery in visited_set:
                related_pairs.append((pickup, delivery))
        PD_pairs_of_each_vehicle.append(related_pairs)

    # 全ての 2車両ペア (i, j) に対し、2車両VRPで最適化した候補を収集
    for i in range(num_vehicles):
        for j in range(i + 1, num_vehicles):
            # 対象ノード集合（両ルートの訪問ノード + 各自デポ）
            combined_node_ids = set(original_routes[i] + original_routes[j])
            if original_routes[i]:
                combined_node_ids.add(original_routes[i][0])
            if original_routes[j]:
                combined_node_ids.add(original_routes[j][0])

            # サブ顧客＆サブPD
            sub_customers = [c for c in customers if c['id'] in combined_node_ids]
            PD_pairs_2v = PD_pairs_of_each_vehicle[i] + PD_pairs_of_each_vehicle[j]

            # 両車両の（開始=終了）デポ
            start_i = original_routes[i][0] if original_routes[i] else customers[0]['id']
            start_j = original_routes[j][0] if original_routes[j] else customers[0]['id']
            start_depots = [start_i, start_j]
            end_depots = [start_i, start_j]

            # 初期ルート（デポを取り除いてヒントにする）
            initial_routes = []
            for r in (original_routes[i], original_routes[j]):
                if len(r) >= 2 and r[0] == r[-1]:
                    initial_routes.append(r[1:-1])
                else:
                    initial_routes.append(r)

            # 2車両の部分問題を解く（解が無ければスキップ）
            new_routes = solve_vrp_flexible(
                sub_customers, initial_routes, PD_pairs_2v,
                num_vehicles=2, vehicle_capacity=vehicle_capacity,
                start_depots=start_depots, end_depots=end_depots,
                use_capacity=True, use_time=True, use_pickup_delivery=True,
                isGAT=True  # ※あなたの実装に合わせています
            )
            if new_routes is None:
                continue

            old_cost = route_cost(original_routes[i], customers) + route_cost(original_routes[j], customers)
            new_cost = sum(route_cost(r, customers) for r in new_routes)

            # 改善がある場合のみ候補として保存
            if new_cost < old_cost:
                feasible_actions.append({
                    'vehicle_pair': (i, j),
                    'new_routes': new_routes,
                    'old_cost': old_cost,
                    'new_cost': new_cost,
                    'cost_improvement': old_cost - new_cost
                })

                # 追加の“入れ替え版”も候補に入れる（元実装の有効手）
                depot_i = new_routes[0][0]
                depot_j = new_routes[1][0]
                mid_i = [n for n in new_routes[0] if n != depot_i]
                mid_j = [n for n in new_routes[1] if n != depot_j]
                exchanged_routes = [
                    [depot_i] + mid_j + [depot_i],
                    [depot_j] + mid_i + [depot_j]
                ]
                exchanged_cost = sum(route_cost(r, customers) for r in exchanged_routes)
                if exchanged_cost < old_cost:
                    feasible_actions.append({
                        'vehicle_pair': (i, j),
                        'new_routes': exchanged_routes,
                        'old_cost': old_cost,
                        'new_cost': exchanged_cost,
                        'cost_improvement': old_cost - exchanged_cost
                    })

    # 改善候補が無ければそのまま返す
    if not feasible_actions:
        return original_routes

    # CP-SAT：各車両は高々1回だけ使われるようにアクションを選択し、総改善量を最大化
    model = cp_model.CpModel()
    x = [model.NewBoolVar(f'action_{k}') for k in range(len(feasible_actions))]
    model.Maximize(sum(int(round(a['cost_improvement'])) * x[k] for k, a in enumerate(feasible_actions)))

    # 各車両が複数アクションで同時に使われないよう制約
    vehicle_to_actions = {}
    for k, a in enumerate(feasible_actions):
        i, j = a['vehicle_pair']
        vehicle_to_actions.setdefault(i, []).append(k)
        vehicle_to_actions.setdefault(j, []).append(k)
    for v, idxs in vehicle_to_actions.items():
        model.Add(sum(x[i] for i in idxs) <= 1)

    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    new_all_vehicles_routes = original_routes.copy()
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        for k, a in enumerate(feasible_actions):
            if solver.Value(x[k]) == 1:
                i, j = a['vehicle_pair']
                r0, r1 = a['new_routes']
                new_all_vehicles_routes[i] = r0
                new_all_vehicles_routes[j] = r1
    else:
        print("最適なアクションの組み合わせが見つかりませんでした。")

    return new_all_vehicles_routes

