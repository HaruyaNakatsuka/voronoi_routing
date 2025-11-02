from parser import parse_lilim200
from flexible_vrp_solver import route_cost
from gat import initialize_individual_vrps, perform_gat_exchange  # 初期解生成/GAT社内最適化で流用
from visualizer import plot_routes
from web_exporter import export_vrp_state, generate_index_json
from voronoi_allocator import perform_voronoi_routing  # ボロノイ再配布＋各社VRP
import time
import os
from itertools import chain


# ==============================
# === テストケースの定義部 ===
# ==============================
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


def compute_company_costs(routes, all_customers, vehicle_num_list):
    """vehicle_num_list に従って routes を会社ごとに分割し、各社の合計 route_cost を返す"""
    costs = []
    vidx = 0
    for n in vehicle_num_list:
        s = 0.0
        for _ in range(n):
            s += route_cost(routes[vidx], all_customers)
            vidx += 1
        costs.append(s)
    return costs


def split_routes_by_company(routes, vehicle_num_list):
    """全車両ルート配列を company ごとのサブ配列に分割"""
    out = []
    idx = 0
    for n in vehicle_num_list:
        out.append(routes[idx: idx + n])
        idx += n
    return out


def flatten(list_of_lists):
    return list(chain.from_iterable(list_of_lists))


def filter_subcustomers_by_routes(all_customers, company_routes):
    """その会社のルートに登場するノードのみを抽出して customers を縮約"""
    node_ids = set()
    for r in company_routes:
        node_ids.update(r)
    return [c for c in all_customers if c["id"] in node_ids]


def filter_pd_pairs_for_nodes(all_PD_pairs, node_ids_set):
    """PD両端が node_ids_set に含まれるペアのみ残す"""
    return {p: d for p, d in all_PD_pairs.items() if p in node_ids_set and d in node_ids_set}


# ==============================
# === テストケースの実行部 ===
# ==============================
for case_index, (file_paths, offsets) in enumerate(test_cases, 1):
    print("\n" + "="*60)
    print(f"テストケース {case_index}: {file_paths[0]} + {file_paths[1]}")
    print(f"オフセット: {offsets[0]} , {offsets[1]}")
    print("="*60)

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

    # === データファイルをパース（座標・IDオフセットを付与し結合）===
    id_offset = 0
    for path, offset in zip(file_paths, offsets):
        data = parse_lilim200(path, x_offset=offset[0], y_offset=offset[1], id_offset=id_offset)

        all_customers.extend(data['customers'])
        all_PD_pairs.update(data['PD_pairs'])
        depot_id_list.append(data['depot_id'])
        depot_coords.append(data['depot_coord'])
        vehicle_num_list.append(data['num_vehicles'])
        num_vehicles += data['num_vehicles']

        max_id = max(c['id'] for c in data['customers'])
        id_offset = max_id + 1

        if vehicle_capacity is None:
            vehicle_capacity = data['vehicle_capacity']

    # =============================
    # === 初期：LSP個別の経路生成 ===
    # =============================
    routes = initialize_individual_vrps(
        all_customers, all_PD_pairs, num_lsps, vehicle_num_list, depot_id_list, vehicle_capacity=vehicle_capacity
    )
    export_vrp_state(all_customers, routes, all_PD_pairs, 0, case_index=case_index,
                     depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,
                     instance_name=instance_name, output_root="web_data")
    plot_routes(all_customers, routes, depot_id_list, vehicle_num_list, iteration=0, instance_name=instance_name)
    # 初期コスト（会社別・総計）
    initial_company_costs = compute_company_costs(routes, all_customers, vehicle_num_list)
    initial_total_cost = sum(initial_company_costs)
    print("\n==== 初期経路：会社別コスト ====")
    for idx, c in enumerate(initial_company_costs, 1):
        print(f"LSP {idx}: {c:.2f}")
    print(f"TOTAL: {initial_total_cost:.2f}")


    # ==========================================
    # === Voronoi再配布 → 各社で一発最適化 ===
    # ==========================================
    print("\n=== Voronoi再配布による経路決定を実行します ===")
    voronoi_routes = perform_voronoi_routing(
        customers=all_customers,
        PD_pairs=all_PD_pairs,
        depot_id_list=depot_id_list,
        vehicle_num_list=vehicle_num_list,
        vehicle_capacity=vehicle_capacity
    )

    # ボロノイ後コスト（会社別・総計）
    voronoi_company_costs = compute_company_costs(voronoi_routes, all_customers, vehicle_num_list)
    voronoi_total_cost = sum(voronoi_company_costs)

    print("\n==== ボロノイ分割後：会社別コスト ====")
    for idx, c in enumerate(voronoi_company_costs, 1):
        print(f"LSP {idx}: {c:.2f}")
    print(f"TOTAL: {voronoi_total_cost:.2f}")

    # 各社改善率（初期 → ボロノイ）
    print("\n==== 各社の改善率（初期 → ボロノイ） ====")
    for idx, (c0, c1) in enumerate(zip(initial_company_costs, voronoi_company_costs), 1):
        improve = ((c0 - c1) / c0 * 100.0) if c0 > 0 else 0.0
        sign = "+" if improve >= 0 else ""
        print(f"LSP {idx}: {sign}{improve:.2f}% ( {c0:.2f} → {c1:.2f} )")

    overall_improve_voronoi = ((initial_total_cost - voronoi_total_cost) / initial_total_cost * 100.0) if initial_total_cost > 0 else 0.0
    sign_total = "+" if overall_improve_voronoi >= 0 else ""
    print("\n==== 全体改善率（初期 → ボロノイ） ====")
    print(f"{sign_total}{overall_improve_voronoi:.2f}% ( {initial_total_cost:.2f} → {voronoi_total_cost:.2f} )")

    # 可視化 & Web出力（Step 1 として保存）
    export_vrp_state(all_customers, voronoi_routes, all_PD_pairs, 1, case_index=case_index,
                     depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,
                     instance_name=instance_name, output_root="web_data")
    plot_routes(all_customers, voronoi_routes, depot_id_list, vehicle_num_list, iteration=1, instance_name=instance_name)

    # =======================================================
    # === 社内限定の GAT 改善（会社ごとに独立に繰り返し） ===
    # =======================================================
    print("\n=== 社内限定GATによる経路改善を実行します（会社横断の交換は行わない） ===")
    per_company_routes = split_routes_by_company(voronoi_routes, vehicle_num_list)
    improved_company_routes = []

    for comp_idx, company_routes in enumerate(per_company_routes):
        # 会社のサブ顧客集合を抽出（その会社ルートに登場するノードのみ）
        sub_customers = filter_subcustomers_by_routes(all_customers, company_routes)
        sub_node_ids = set(c["id"] for c in sub_customers)
        # PDペアも社内分に限定
        sub_PD_pairs_dict = filter_pd_pairs_for_nodes(all_PD_pairs, sub_node_ids)

        # 社内GATループ
        prev_cost = sum(route_cost(r, all_customers) for r in company_routes)
        iter_count = 1
        while True:
            new_company_routes = perform_gat_exchange(
                company_routes,          # ← 会社内ルートのみ
                sub_customers,           # ← 会社内の顧客のみ
                sub_PD_pairs_dict,       # ← 会社内のPDのみ
                vehicle_capacity,
                [len(company_routes)]    # ← この会社の台数のみ
            )
            new_cost = sum(route_cost(r, all_customers) for r in new_company_routes)
            from_prev = ((prev_cost - new_cost) / prev_cost * 100.0) if prev_cost > 0 else 0.0

            # 会社内の改善が小さくなったら終了（0.0% 切り上げ判定）
            if round(from_prev, 1) == 0.0:
                company_routes = new_company_routes
                break
            else:
                company_routes = new_company_routes
                prev_cost = new_cost
                iter_count += 1

        improved_company_routes.append(company_routes)

    # 全社の最終ルートを連結
    gat_final_routes = flatten(improved_company_routes)

    # GAT後コスト（会社別・総計）
    gat_company_costs = compute_company_costs(gat_final_routes, all_customers, vehicle_num_list)
    gat_total_cost = sum(gat_company_costs)

    print("\n==== GAT改善後（社内限定）：会社別コスト ====")
    for idx, c in enumerate(gat_company_costs, 1):
        print(f"LSP {idx}: {c:.2f}")
    print(f"TOTAL: {gat_total_cost:.2f}")

    # 各社改善率（ボロノイ → GAT）
    print("\n==== 各社の改善率（ボロノイ → GAT） ====")
    for idx, (c0, c1) in enumerate(zip(voronoi_company_costs, gat_company_costs), 1):
        improve = ((c0 - c1) / c0 * 100.0) if c0 > 0 else 0.0
        sign = "+" if improve >= 0 else ""
        print(f"LSP {idx}: {sign}{improve:.2f}% ( {c0:.2f} → {c1:.2f} )")

    overall_improve_gat = ((voronoi_total_cost - gat_total_cost) / voronoi_total_cost * 100.0) if voronoi_total_cost > 0 else 0.0
    sign_total_gat = "+" if overall_improve_gat >= 0 else ""
    print("\n==== 全体改善率（ボロノイ → GAT） ====")
    print(f"{sign_total_gat}{overall_improve_gat:.2f}% ( {voronoi_total_cost:.2f} → {gat_total_cost:.2f} )")

    # 追加の可視化 & Web出力（Step 2 として保存）
    export_vrp_state(all_customers, gat_final_routes, all_PD_pairs, 2, case_index=case_index,
                     depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,
                     instance_name=instance_name, output_root="web_data")
    plot_routes(all_customers, gat_final_routes, depot_id_list, vehicle_num_list, iteration=2, instance_name=instance_name)

    # === React側へ今回のインスタンスだけ反映 ===
    generate_index_json(instance_name=instance_name, output_root="web_data", target_root="vrp-viewer/public/vrp_data")

    # 実行時間
    elapsed = time.time() - start_time
    print(f"\n=== テストケース {case_index} の実行時間: {elapsed:.2f} 秒 ===")
