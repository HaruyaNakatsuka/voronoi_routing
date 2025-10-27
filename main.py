from parser import parse_lilim200
from flexible_vrp_solver import route_cost
from gat import initialize_individual_vrps, perform_gat_exchange
from visualizer import plot_routes
from web_exporter import export_vrp_state, generate_index_json
import time
import os


# ==============================
# === テストケースの定義部 ===
# ==============================
"""
test_cases = [
    (["data/LC1_2_2.txt", "data/LC1_2_6.txt"], [(0, 0), (42, -42)]),
    (["data/LC1_2_2.txt", "data/LC1_2_7.txt"], [(0, 0), (-32, -32)]),
    (["data/LC1_2_4.txt", "data/LC1_2_7.txt"], [(0, 0), (-30, 0)]),
    (["data/LC1_2_4.txt", "data/LC1_2_8.txt"], [(0, 0), (-30, 0)]),
    (["data/LC1_2_10.txt", "data/LC1_2_4.txt"], [(0, 0), (30, 0)]),
    (["data/LR1_2_3.txt", "data/LR1_2_8.txt"], [(0, 0), (0, 30)]),
    (["data/LR1_2_5.txt", "data/LR1_2_8.txt"], [(0, 0), (0, 30)]),
    (["data/LR1_2_8.txt", "data/LR1_2_9.txt"], [(0, 0), (0, -30)]),
    (["data/LR1_2_10.txt", "data/LR1_2_3.txt"], [(0, 0), (0, -30)]),
    (["data/LR1_2_10.txt", "data/LR1_2_8.txt"], [(0, 0), (0, 30)])
]
"""

test_cases = [
    (["data/LR1_2_10.txt", "data/LR1_2_8.txt"], [(0, 0), (0, 30)]),
    (["data/LR1_2_10.txt", "data/test1.txt"], [(0, 0), (0, 30)])
]

# ==============================
# === テストケースの実行部 ===
# ==============================
for case_index, (file_paths, offsets) in enumerate(test_cases, 1):
    print("\n" + "="*50)
    print(f"テストケース {case_index}: {file_paths[0]} + {file_paths[1]}")
    print(f"オフセット: {offsets[0]} , {offsets[1]}")
    print("="*50)
    
    instance_name = f"{os.path.basename(file_paths[0]).split('.')[0]}_{os.path.basename(file_paths[1]).split('.')[0]}"
    
    start_time = time.time()

    num_lsps = len(file_paths)
    num_vehicles = 0
    all_customers = []
    all_PD_pairs = {}
    depot_id_list = []
    depot_coords = []
    vehicle_num_list = []
    vehicle_capacity = None

    # === データファイルをパース ===
    id_offset = 0  # 初期IDオフセット
    for path, offset in zip(file_paths, offsets):
        data = parse_lilim200(path, x_offset=offset[0], y_offset=offset[1], id_offset=id_offset)

        # データ蓄積
        all_customers.extend(data['customers'])
        all_PD_pairs.update(data['PD_pairs'])
        depot_id_list.append(data['depot_id'])
        depot_coords.append(data['depot_coord'])
        vehicle_num_list.append(data['num_vehicles'])
        num_vehicles += data['num_vehicles']
        
        # IDオフセットを次に備えて更新
        max_id = max(c['id'] for c in data['customers'])
        id_offset = max_id + 1

        # 車両容量の情報を保存（全ファイルで同じ前提）
        if vehicle_capacity is None:
            vehicle_capacity = data['vehicle_capacity']


    #      =============================
    #      === LSP個別経路生成フェーズ ===
    #      =============================
    routes = initialize_individual_vrps(
        all_customers, all_PD_pairs, num_lsps, vehicle_num_list, depot_id_list, vehicle_capacity=vehicle_capacity
    )
    plot_routes(all_customers, routes, depot_id_list, vehicle_num_list, iteration=0, instance_name=instance_name)
    export_vrp_state(all_customers, routes, all_PD_pairs, 0, case_index,
                 depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list)
    
    initial_cost = sum(route_cost(route, all_customers) for route in routes)
    print(f"初期経路コスト＝{initial_cost}")
    previous_cost = initial_cost
    

    def print_routes_with_lsp_separator(routes, vehicle_num_list):
        vehicle_index = 0
        for lsp_index, num_vehicles in enumerate(vehicle_num_list):
            print(f"--- LSP {lsp_index + 1} ---")
            for _ in range(num_vehicles):
                route = routes[vehicle_index]
                print(f"  Vehicle {vehicle_index + 1}: {' -> '.join(map(str, route))}")
                vehicle_index += 1
                
    def compute_company_costs(routes, all_customers, vehicle_num_list):
        """各LSPごとの総コストを計算する"""
        company_costs = []
        vehicle_index = 0
        for num_vehicles in vehicle_num_list:
            lsp_cost = 0
            for _ in range(num_vehicles):
                lsp_cost += route_cost(routes[vehicle_index], all_customers)
                vehicle_index += 1
            company_costs.append(lsp_cost)
        return company_costs

    current_company_costs = compute_company_costs(routes, all_customers, vehicle_num_list)
    previous_company_costs = current_company_costs.copy()
    print("---- 各LSPのコスト ----")
    for idx, (prev_c, curr_c) in enumerate(zip(previous_company_costs, current_company_costs), 1):
        improvement = (prev_c - curr_c) / prev_c * 100 if prev_c != 0 else 0
        print(f"LSP {idx}: {curr_c:.2f} （前回比 {improvement:+.2f}%）")
    print("-------------------------\n")


    #print("=== 初期経路 ===")  
    #print_routes_with_lsp_separator(routes, vehicle_num_list)


    #       ==========================
    #       ===== GAT改善フェーズ =====
    #       ==========================
    i=1
    while True:
        print(f"=== gat改善：{i}回目 ===")
        
        routes = perform_gat_exchange(
            routes, all_customers, all_PD_pairs, vehicle_capacity, vehicle_num_list
        )
        plot_routes(all_customers, routes, depot_id_list, vehicle_num_list, iteration=i, instance_name=instance_name)
        export_vrp_state(all_customers, routes, all_PD_pairs, i, case_index,
                 depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list)
        #print_routes_with_lsp_separator(routes, vehicle_num_list)
        
                # === 各LSPごとのコスト計算 ===
        current_company_costs = compute_company_costs(routes, all_customers, vehicle_num_list)
        
        print("---- 各LSPのコスト ----")
        for idx, (prev_c, curr_c) in enumerate(zip(previous_company_costs, current_company_costs), 1):
            improvement = (prev_c - curr_c) / prev_c * 100 if prev_c != 0 else 0
            print(f"LSP {idx}: {curr_c:.2f} （前回比 {improvement:+.2f}%）")
        print("-------------------------\n")

        # 次回比較用に保存
        previous_company_costs = current_company_costs.copy()

        # コスト改善率計算
        current_cost = sum(route_cost(route, all_customers) for route in routes)
        from_initial = (initial_cost - current_cost) / initial_cost * 100
        from_previous = (previous_cost - current_cost) / previous_cost * 100
        #print(f"[初期ルートからのコスト改善率] {from_initial:.2f}%")
        #print(f"[前回経路からのコスト改善率] {from_previous:.2f}%")
        if round(from_previous, 1) == 0.0:
            print(f"最終コスト＝{current_cost}")
            print(f"初期ルートからのコスト改善率＝ {from_initial:.2f}%")
            break
        else:
            previous_cost = current_cost
            i=i+1

    # 経路改善終了, 実行時間表示
    end_time = time.time()
    elapsed = end_time - start_time
    print(f"=== テストケース {case_index} の実行時間: {elapsed:.2f} 秒 ===")
    
generate_index_json()
