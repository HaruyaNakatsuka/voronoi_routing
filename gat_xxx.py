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
            isInitPhase=True
        )

        all_vehicle_routes.extend(lsp_routes)

    return all_vehicle_routes


def perform_gat_exchange(original_routes, customers, PD_pairs, vehicle_capacity):
    feasible_actions = []#実行可能アクション集合
    num_vehicles = len(original_routes)
    
    #各車両ごとの集荷->配達のペアをまとめたリストを作成
    PD_pairs_of_each_vehicle = []
    for vehicle_route in original_routes:
        related_pairs = []
        visited_set = set(vehicle_route)
        for pickup, delivery in PD_pairs.items():
            if pickup in visited_set or delivery in visited_set:
                related_pairs.append((pickup, delivery))
        PD_pairs_of_each_vehicle.append(related_pairs)
        
    #全2車両ペアに対して2車両VRPを実行
    for i in range(num_vehicles):
        for j in range(i + 1, num_vehicles):
            #print(f">車両No.{i+1:3}と車両No.{j+1:3}のGAT検証・・・")

            # 2車両分の訪問地点（空リストも考慮）を結合して集合に
            combined_node_ids = set(original_routes[i] + original_routes[j])
            
            # 両車両のデポは必ず含める
            combined_node_ids.add(original_routes[i][0])
            combined_node_ids.add(original_routes[j][0])

            # 該当する顧客情報を抽出
            sub_customers = [c for c in customers if c['id'] in combined_node_ids]

            # 該当するpickup→deliveryタプルを結合
            PD_pairs_of_2vehicle = PD_pairs_of_each_vehicle[i] + PD_pairs_of_each_vehicle[j]
            
            # デポ情報の抽出
            start_depots = [original_routes[i][0], original_routes[j][0]]
            end_depots = [original_routes[i][0], original_routes[j][0]]

            # routing.ReadAssignmentFromRoutes用引数
            initial_routes = [r[1:-1] for r in [original_routes[i], original_routes[j]]]

            #2車両VRP解決
            new_routes = solve_vrp_flexible(sub_customers, initial_routes, PD_pairs_of_2vehicle, 2, vehicle_capacity, start_depots, end_depots,
                                            use_capacity=True, use_time=True, use_pickup_delivery=True, isInitPhase=False)
            old_cost = route_cost(original_routes[i], customers) + route_cost(original_routes[j], customers)
            new_cost = sum(route_cost(r, customers) for r in new_routes)
            #経路が更新されていればアクション集合に追加
            if new_cost < old_cost:
                feasible_actions.append({
                    'vehicle_pair': (i, j),
                    'old_routes': [original_routes[i], original_routes[j]],
                    'new_routes': new_routes,
                    'old_cost': old_cost,
                    'new_cost': new_cost,
                    'cost_improvement': old_cost - new_cost
                })
                # 非効率な経路交換：ルート本体だけを入れ替え、デポはそれぞれの元のデポを維持する
                original_depot_i = new_routes[0][0]
                original_depot_j = new_routes[1][0]
                # 中間ノード（デポ除く）を抽出
                mid_i = [n for n in new_routes[0] if n != original_depot_i]
                mid_j = [n for n in new_routes[1] if n != original_depot_j]
                # 丸ごと交換した新しい経路を作成（デポ固定）
                exchanged_routes = [
                [original_depot_i] + mid_j + [original_depot_i],
                [original_depot_j] + mid_i + [original_depot_j]
                ]
                exchanged_cost = sum(route_cost(r, customers) for r in exchanged_routes)
                #アクション集合に追加
                feasible_actions.append({
                    'vehicle_pair': (i, j),
                    'old_routes': [original_routes[i], original_routes[j]],
                    'new_routes': exchanged_routes,
                    'old_cost': old_cost,
                    'new_cost': exchanged_cost,
                    'cost_improvement': old_cost - exchanged_cost
                })

    #アクション集合の中からコストが最も改善する経路交換を決定する↓
    model = cp_model.CpModel()# OR-Tools CP-SAT Solver を使って最適なアクション集合を選択
    num_actions = len(feasible_actions)
    x = [model.NewBoolVar(f'action_{i}') for i in range(num_actions)]# アクションごとの選択変数
    model.Maximize(sum(x[i] * feasible_actions[i]['cost_improvement'] for i in range(num_actions)))# 目的関数：cost_improvementの合計を最大化

    # 各車両について、1回しか使われてはいけないという制約
    vehicle_to_actions = {}
    for i, action in enumerate(feasible_actions):
        v1, v2 = action['vehicle_pair']
        for v in [v1, v2]:
            if v not in vehicle_to_actions:
                vehicle_to_actions[v] = []
            vehicle_to_actions[v].append(i)
    for v, action_indices in vehicle_to_actions.items():
        model.Add(sum(x[i] for i in action_indices) <= 1)

    # ソルバーで最適化
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    # 最終ルートの更新
    new_all_vehicles_routes = original_routes.copy()
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for i in range(num_actions):
            if solver.Value(x[i]) == 1:
                v1, v2 = feasible_actions[i]['vehicle_pair']
                new_routes = feasible_actions[i]['new_routes']
                new_all_vehicles_routes[v1] = new_routes[0]
                new_all_vehicles_routes[v2] = new_routes[1]
    else:
        print("最適なアクションの組み合わせが見つかりませんでした。")

    return new_all_vehicles_routes