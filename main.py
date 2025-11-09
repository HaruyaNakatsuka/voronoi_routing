from parser import parse_lilim200
from flexible_vrp_solver import route_cost
from gat import initialize_individual_vrps, perform_gat_exchange  # 初期解生成/GAT社内最適化で流用
from visualizer import plot_routes
from web_exporter import export_vrp_state, generate_index_json
from voronoi_allocator import perform_voronoi_routing  # ボロノイ再配布＋各社VRP
import time
import os
from itertools import chain
import logging
from tabulate import tabulate


# ============ 出力ON/OFFフラグ（環境変数でも制御可。未設定ならON） =========================
ENABLE_EXPORT = os.getenv("VRP_ENABLE_EXPORT", "1") == "1"  # JSON出力(export_vrp_state)
ENABLE_PLOT   = os.getenv("VRP_ENABLE_PLOT",   "1") == "1"  # PNG出力(plot_routes)
# =======================================================================================


def setup_logging(show_progress: bool = True):
    logging.basicConfig(
        level=logging.INFO if show_progress else logging.WARNING,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        force=True,  # ← これがポイント
    )
    
if __name__ == "__main__":
    setup_logging(show_progress=False)  # Falseにするとprint類がすべて非表示に


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


# ==============================
# === テストケースの実行部 ===
# ==============================
for case_index, (file_paths, offsets) in enumerate(test_cases, 1):
    print("\n\n" + "="*60)
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
    
    #　[コンソール出力] -> 会社別コスト
    initial_company_costs = compute_company_costs(routes, all_customers, vehicle_num_list)
    initial_total_cost = sum(initial_company_costs)
    print("\n==== 初期経路：会社別コスト ====")
    for idx, c in enumerate(initial_company_costs, 1):
        print(f"LSP {idx}: {c:.2f}")
    print(f"TOTAL: {initial_total_cost:.2f}")
    # [データ保存] -> jsonファイル、pngファイル
    if ENABLE_EXPORT:
        export_vrp_state(all_customers, routes, all_PD_pairs, 0, case_index=case_index,depot_id_list=depot_id_list,
                    vehicle_num_list=vehicle_num_list,instance_name=instance_name, output_root="web_data")
    if ENABLE_PLOT:
        plot_routes(all_customers, routes, depot_id_list, vehicle_num_list, iteration=0, instance_name=instance_name)


    # ==========================================
    # === Voronoi再配布 → 各社で一発最適化 ===
    # ==========================================
    print("\n=== Voronoi分割による経路再生成 ===")
    voronoi_routes = perform_voronoi_routing(
        customers=all_customers,
        PD_pairs=all_PD_pairs,
        depot_id_list=depot_id_list,
        vehicle_num_list=vehicle_num_list,
        vehicle_capacity=vehicle_capacity
    )

    #　[コンソール出力] -> 改善率、他
    voronoi_company_costs = compute_company_costs(voronoi_routes, all_customers, vehicle_num_list)
    voronoi_total_cost = sum(voronoi_company_costs)
    colw = 10
    print(
        " " * 7 +
        "{:>{w}} {:>{w}} {:>{w}}".format(
            "初期コスト", "暫定コスト", "初期比改善(%)", w=colw
        )
    )
    colw = 15
    for idx, (init_c, cur_c) in enumerate(zip(initial_company_costs, voronoi_company_costs), 1):
        init_improve = ((init_c - cur_c) / init_c * 100.0) if init_c > 0 else 0.0
        print(
            f"LSP {idx:<2} " +
            "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
                init_c, cur_c, init_improve, w=colw
            )
        )
    init_improve_total = ((initial_total_cost - voronoi_total_cost) / initial_total_cost * 100.0) if initial_total_cost > 0 else 0.0
    print(
        f"{'TOTAL':<6} " +
        "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
            initial_total_cost, voronoi_total_cost, init_improve_total, w=colw
        )
    )
    # [データ保存] -> jsonファイル、pngファイル
    if ENABLE_EXPORT:
        export_vrp_state(all_customers, voronoi_routes, all_PD_pairs, 1, case_index=case_index,
                     depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,
                     instance_name=instance_name, output_root="web_data")
    if ENABLE_PLOT:
        plot_routes(all_customers, voronoi_routes, depot_id_list, vehicle_num_list, iteration=1, instance_name=instance_name)

    # =======================================================
    # === 社内限定の GAT 改善（会社ごとに独立に繰り返し） ===
    # =======================================================
    print("\n=== 社内GATによる経路改善 ===")

    num_companies = len(vehicle_num_list)
    converged = [False] * num_companies          # 会社ごとの収束フラグ
    gat_round = 1
    gat_current_routes = voronoi_routes[:]       # 作業用コピー
    step_idx = 2                                 # 0=初期, 1=ボロノイ, 以降はGATラウンド

    while not all(converged):
        print(f"--- 社内GATラウンド {gat_round} ---")

        prev_company_costs = compute_company_costs(gat_current_routes, all_customers, vehicle_num_list)
        prev_total_cost = sum(prev_company_costs)

        # 会社ごとにルートを分割
        per_company_routes = split_routes_by_company(gat_current_routes, vehicle_num_list)
        next_company_routes_list = []

        for comp_idx, company_routes in enumerate(per_company_routes):
            # 収束済みの会社はスキップ
            if converged[comp_idx]:
                print(f">>> LSP {comp_idx + 1}: 収束済みのためスキップ")
                next_company_routes_list.append(company_routes)
                continue

            # 社内の顧客/PDに絞る
            sub_customers = filter_subcustomers_by_routes(all_customers, company_routes)
            sub_node_ids = set(c["id"] for c in sub_customers)
            sub_PD_pairs_dict = filter_pd_pairs_for_nodes(all_PD_pairs, sub_node_ids)

            # 社内GATを1回実行
            old_cost_company = sum(route_cost(r, all_customers) for r in company_routes)
            new_company_routes = perform_gat_exchange(
                company_routes,                # 会社内ルートのみ
                sub_customers,                 # 会社内顧客のみ
                sub_PD_pairs_dict,             # 会社内PDのみ
                vehicle_capacity,
                [len(company_routes)]          # その会社の台数のみ
            )
            new_cost_company = sum(route_cost(r, all_customers) for r in new_company_routes)

            # 改善判定（数値ゆらぎ対策）
            if new_cost_company + 1e-9 < old_cost_company:
                delta = (old_cost_company - new_cost_company) / old_cost_company * 100.0 if old_cost_company > 0 else 0.0
                print(f">>> LSP {comp_idx + 1}: +{delta:.2f}%  ( {old_cost_company:.2f} → {new_cost_company:.2f} )")
                next_company_routes_list.append(new_company_routes)
                # 改善した会社は次ラウンドも対象（converged は据え置き False）
            else:
                print(f">>> LSP {comp_idx + 1}: +0.00% ( {old_cost_company:.2f} → {new_cost_company:.2f} ) → 収束")
                converged[comp_idx] = True
                next_company_routes_list.append(company_routes)  # 変化なしを引き継ぐ

        # ラウンド結果を反映
        gat_current_routes = flatten(next_company_routes_list)

        #　[コンソール出力] -> 改善率、他
        curr_company_costs = compute_company_costs(gat_current_routes, all_customers, vehicle_num_list)
        curr_total_cost = sum(curr_company_costs)
        colw = 10
        # ヘッダー行
        print(
            " " * 7 +
            "{:>{w}} {:>{w}} {:>{w}} {:>{w}}".format(
                "初期コスト", "暫定コスト", "ラウンド改善(%)", "初期比改善(%)", w=colw
            )
        )
        # 各社の行
        colw = 15
        for idx, (init_c, prev_c, cur_c) in enumerate(zip(initial_company_costs, prev_company_costs, curr_company_costs), 1):
            round_improve = ((prev_c - cur_c) / prev_c * 100.0) if prev_c > 0 else 0.0
            init_improve = ((init_c - cur_c) / init_c * 100.0) if init_c > 0 else 0.0
            print(
                f"LSP {idx:<2} " +
                "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
                    init_c, cur_c, round_improve, init_improve, w=colw
                )
            )
        # TOTAL行
        round_improve_total = ((prev_total_cost - curr_total_cost) / prev_total_cost * 100.0) if prev_total_cost > 0 else 0.0
        init_improve_total = ((initial_total_cost - curr_total_cost) / initial_total_cost * 100.0) if initial_total_cost > 0 else 0.0
        print(
            f"{'TOTAL':<6} " +
            "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
                initial_total_cost, curr_total_cost, round_improve_total, init_improve_total, w=colw
            )
        )

        # [データ保存] -> jsonファイル、pngファイル
        if ENABLE_EXPORT:
            export_vrp_state(all_customers, gat_current_routes, all_PD_pairs, step_idx,case_index=case_index,
                         depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,instance_name=instance_name, output_root="web_data")
        if ENABLE_PLOT:
            plot_routes(all_customers, gat_current_routes, depot_id_list, vehicle_num_list,iteration=step_idx, instance_name=instance_name)

        # 次のラウンドへ
        gat_round += 1
        step_idx += 1

    print("\n>>> 全社が収束（改善率=0%）したため、社内GATを終了")

    #  [データ保存] -> jsonファイル
    if ENABLE_EXPORT:
        generate_index_json(instance_name=instance_name, output_root="web_data", target_root="vrp-viewer/public/vrp_data")

    # 実行時間
    elapsed = time.time() - start_time
    print(f">>> テストケース {case_index} の実行時間: {elapsed:.2f} 秒")
