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


def perform_gat_exchange(original_routes, customers, PD_pairs, vehicle_capacity, vehicle_num_list):
    feasible_actions = []  # 実行可能アクション集合
    num_vehicles = len(original_routes)
    
    # --- 追加: 各車両 -> 会社 の単純マッピングを作る（vehicle_num_list に基づく） ---
    vehicle_to_company = []
    for comp_idx, n in enumerate(vehicle_num_list):
        for _ in range(n):
            vehicle_to_company.append(comp_idx)
    
    num_companies = max(vehicle_to_company) + 1 if vehicle_to_company else 1

    # --- 各車両ごとの集荷->配達のペアをまとめたリストを作成（既存） ---
    PD_pairs_of_each_vehicle = []
    for vehicle_route in original_routes:
        related_pairs = []
        visited_set = set(vehicle_route)
        for pickup, delivery in PD_pairs.items():
            if pickup in visited_set or delivery in visited_set:
                related_pairs.append((pickup, delivery))
        PD_pairs_of_each_vehicle.append(related_pairs)
        
    # --- 全2車両ペアに対して2車両VRPを実行（候補収集） ---
    for i in range(num_vehicles):
        for j in range(i + 1, num_vehicles):
            # 2車両分の訪問地点（空リストも考慮）を結合して集合に
            combined_node_ids = set(original_routes[i] + original_routes[j])
            
            # 両車両のデポは必ず含める（既存ロジック）
            if original_routes[i]:
                combined_node_ids.add(original_routes[i][0])
            if original_routes[j]:
                combined_node_ids.add(original_routes[j][0])

            # 該当する顧客情報を抽出
            sub_customers = [c for c in customers if c['id'] in combined_node_ids]

            # 該当するpickup→deliveryタプルを結合
            PD_pairs_of_2vehicle = PD_pairs_of_each_vehicle[i] + PD_pairs_of_each_vehicle[j]
            
            # デポ情報の抽出
            start_depots = [original_routes[i][0] if original_routes[i] else customers[0]['id'],
                            original_routes[j][0] if original_routes[j] else customers[0]['id']]
            end_depots = [start_depots[0], start_depots[1]]

            # routing.ReadAssignmentFromRoutes用引数（デポ除去）
            initial_routes = [r[1:-1] if len(r) >= 2 and r[0] == r[-1] else r for r in [original_routes[i], original_routes[j]]]

            # 2車両VRP解決（既存関数を呼ぶ）
            new_routes = solve_vrp_flexible(sub_customers, initial_routes, PD_pairs_of_2vehicle,
                                            2, vehicle_capacity, start_depots, end_depots,
                                            use_capacity=True, use_time=True, use_pickup_delivery=True, isInitPhase=False)
            
            # VRPが解決されなかったら疑似的に元の経路をアクション集合に保存
            if new_routes is None:
                new_routes = [original_routes[i], original_routes[j]]  # 疑似的に元ルートを代入
                old_cost = route_cost(original_routes[i], customers) + route_cost(original_routes[j], customers)
                new_cost = old_cost  # 変化なし
                delta_per_company = [0.0] * num_companies
                feasible_actions.append({
                    'vehicle_pair': (i, j),
                    'old_routes': [original_routes[i], original_routes[j]],
                    'new_routes': new_routes,
                    'old_cost': old_cost,
                    'new_cost': new_cost,
                    'cost_improvement': 0,
                    'delta_per_company': delta_per_company
                })
                # 非効率交換はスキップ
                continue

            old_cost = route_cost(original_routes[i], customers) + route_cost(original_routes[j], customers)
            new_cost = sum(route_cost(r, customers) for r in new_routes)
            # コスト(経路長)が改善されていればアクション集合に追加（既存条件）
            if new_cost < old_cost:
                
                delta_per_company = [0.0] * num_companies
                old_i = route_cost(original_routes[i], customers)
                new_i = route_cost(new_routes[0], customers)
                delta_per_company[vehicle_to_company[i]] += (new_i - old_i)
                old_j = route_cost(original_routes[j], customers)
                new_j = route_cost(new_routes[1], customers)
                delta_per_company[vehicle_to_company[j]] += (new_j - old_j)
    
                feasible_actions.append({
                    'vehicle_pair': (i, j),
                    'old_routes': [original_routes[i], original_routes[j]],
                    'new_routes': new_routes,
                    'old_cost': old_cost,
                    'new_cost': new_cost,
                    'cost_improvement': old_cost - new_cost,
                    'delta_per_company': delta_per_company  # ← 追加情報を保持
                })
                # 非効率な経路交換（元の実装）：
                original_depot_i = new_routes[0][0]
                original_depot_j = new_routes[1][0]
                mid_i = [n for n in new_routes[0] if n != original_depot_i]
                mid_j = [n for n in new_routes[1] if n != original_depot_j]
                exchanged_routes = [
                    [original_depot_i] + mid_j + [original_depot_i],
                    [original_depot_j] + mid_i + [original_depot_j]
                ]
                exchanged_cost = sum(route_cost(r, customers) for r in exchanged_routes)
                # 交換時の delta（簡潔に再計算）
                exchanged_delta = [0.0] * num_companies
                ex_new_i = route_cost(exchanged_routes[0], customers)
                ex_new_j = route_cost(exchanged_routes[1], customers)
                exchanged_delta[vehicle_to_company[i]] += (ex_new_i - old_i)
                exchanged_delta[vehicle_to_company[j]] += (ex_new_j - old_j)
                feasible_actions.append({
                    'vehicle_pair': (i, j),
                    'old_routes': [original_routes[i], original_routes[j]],
                    'new_routes': exchanged_routes,
                    'old_cost': old_cost,
                    'new_cost': exchanged_cost,
                    'cost_improvement': old_cost - exchanged_cost,
                    'delta_per_company': exchanged_delta
                })

    # --- CP-SAT モデル構築（最小限の変更） ---
    model = cp_model.CpModel()
    num_actions = len(feasible_actions)
    x = [model.NewBoolVar(f'action_{k}') for k in range(num_actions)]
    # 簡潔化のためスケール = 100 を使って浮動小数を整数係数へ（最小の追加）
    model.Maximize(sum(int(round(feasible_actions[k]['cost_improvement'])) * x[k]
                   for k in range(num_actions)))

    # 各車両について 1 回しか使われてはいけない制約（既存）
    vehicle_to_actions = {}
    for k, action in enumerate(feasible_actions):
        v1, v2 = action['vehicle_pair']
        for v in [v1, v2]:
            vehicle_to_actions.setdefault(v, []).append(k)
    for v, idxs in vehicle_to_actions.items():
        model.Add(sum(x[i] for i in idxs) <= 1)

    # --- 追加: 各会社ごとの個別合理性制約（最小限の一文追加） ---
    #  sum_over_actions x[a] * delta[a][company] <= 0  を追加
    for c in range(num_companies):
        terms = []
        for k, action in enumerate(feasible_actions):
            d = action['delta_per_company'][c]
            if abs(d) < 1e-9:
                continue
            terms.append(int(round(d)) * x[k])
        if terms:
            model.Add(sum(terms) <= 0)

    # ソルバーで最適化（既存）
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    # 最終ルートの更新（既存）
    new_all_vehicles_routes = original_routes.copy()
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(num_actions):
            if solver.Value(x[k]) == 1:
                v1, v2 = feasible_actions[k]['vehicle_pair']
                new_routes = feasible_actions[k]['new_routes']
                new_all_vehicles_routes[v1] = new_routes[0]
                new_all_vehicles_routes[v2] = new_routes[1]
    else:
        print("最適なアクションの組み合わせが見つかりませんでした。")

    return new_all_vehicles_routes
