from parser import parse_lilim200
from flexible_vrp_solver import solve_vrp_flexible, route_cost
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

import math

def rank_pd_pairs_by_midpoint_to_voronoi_boundary(all_customers, PD_pairs, depot_id_list):
    """
    PDペアの「重心→最寄りボロノイ境界」距離で昇順に並べる。
    戻り値: list[dict] （距離が小さい順）
      - pickup, delivery: ノードID
      - midpoint: (x, y)
      - d1, d2: 重心から最短/次点デポまでの距離
      - dist_to_boundary: (d2 - d1) / 2
      - nearest_depot, second_nearest_depot: デポID
    """
    # ID -> (x, y)
    id2xy = {c["id"]: (c["x"], c["y"]) for c in all_customers}

    # デポの座標リスト
    depot_xy = [(d, id2xy[d]) for d in depot_id_list]

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    boundary_dist_ranked_PDpairs = []
    for pu, de in PD_pairs.items():
        if pu not in id2xy or de not in id2xy:
            continue  # 安全側
        x1, y1 = id2xy[pu]
        x2, y2 = id2xy[de]
        mid = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)

        # 重心から各デポへの距離を計算して昇順に
        dists = []
        for depot_id, xy in depot_xy:
            mid_to_depo_distance = math.hypot(mid[0] - xy[0], mid[1] - xy[1])
            dists.append((mid_to_depo_distance, depot_id))
        dists.sort(key=lambda t: t[0])

        # 一番近い＆二番目に近いデポ
        d1, dep1 = dists[0]
        d2, dep2 = dists[1] if len(dists) > 1 else (float("inf"), None)

        # ボロノイ境界（等距離線）までの最短距離
        dist_to_boundary = (d2 - d1) * 0.5 if math.isfinite(d2) else float("inf")

        boundary_dist_ranked_PDpairs.append({
            "pickup": pu,
            "delivery": de,
            "midpoint": mid,
            "d1": d1,
            "d2": d2,
            "dist_to_boundary": dist_to_boundary,
            "nearest_depot": dep1,
            "second_nearest_depot": dep2,
        })

    # 境界への距離で昇順ソート
    boundary_dist_ranked_PDpairs.sort(key=lambda r: r["dist_to_boundary"])

    return boundary_dist_ranked_PDpairs


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


def find_company_owning_pd_pair(routes_all, vehicle_num_list, pd_nodes):
    """
    PDノードのいずれか（または両方）を含む会社 index を返す。
    routes_all: すべての車両ルートのリスト
    vehicle_num_list: 各会社の車両数リスト
    pd_nodes: {pickup_id, delivery_id}
    """
    pd_set = set(pd_nodes)
    start = 0
    for comp_idx, nveh in enumerate(vehicle_num_list):
        end = start + nveh
        company_routes = routes_all[start:end]
        # 各ルートにPDノードのどれかが含まれているかを調べる
        if any(any(n in pd_set for n in r) for r in company_routes):
            return comp_idx
        start = end
    return None


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
    routes = perform_voronoi_routing(
        customers=all_customers,
        PD_pairs=all_PD_pairs,
        depot_id_list=depot_id_list,
        vehicle_num_list=vehicle_num_list,
        vehicle_capacity=vehicle_capacity
    )
    current_company_costs = compute_company_costs(routes, all_customers, vehicle_num_list)
    current_total_cost = sum(current_company_costs)
    cost_reduction_rates = [((init_c - cur_c) / init_c * 100.0) for init_c, cur_c 
                                    in zip(initial_company_costs, current_company_costs)]
    total_cost_reduction_rates = ((initial_total_cost - current_total_cost) / initial_total_cost * 100.0)

    #　[コンソール出力] -> 改善率、他
    colw = 10
    print(
        " " * 7 +
        "{:>{w}} {:>{w}} {:>{w}}".format(
            "初期コスト", "暫定コスト", "初期比改善(%)", w=colw
        )
    )
    colw = 15
    for idx, (init_c, cur_c, init_improve) in enumerate(
        zip(initial_company_costs, current_company_costs, cost_reduction_rates), 1
    ):
        print(
            f"LSP {idx:<2} " +
            "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
                init_c, cur_c, init_improve, w=colw
            )
        )
    print(
        f"{'TOTAL':<6} " +
        "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
            initial_total_cost, current_total_cost, total_cost_reduction_rates, w=colw
        )
    )
    
    # [データ保存] -> jsonファイル、pngファイル
    if ENABLE_EXPORT:
        export_vrp_state(all_customers, routes, all_PD_pairs, 1, case_index=case_index,
                     depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,
                     instance_name=instance_name, output_root="web_data")
    if ENABLE_PLOT:
        plot_routes(all_customers, routes, depot_id_list, vehicle_num_list, iteration=1, instance_name=instance_name)
    
    # =======================================================
    # =============== 段階的なタスク再交換===============
    # =======================================================
    id2cust = {c["id"]: c for c in all_customers}
    #depot_coords = {dep_id: (id2cust[dep_id]["x"], id2cust[dep_id]["y"]) for dep_id in depot_id_list}

    def nearest_and_second_depot_company_of_midpoint(pick_id, deliv_id):
        """PD中点からの距離で最寄り/次点デポの会社 index を返す"""
        px, py = id2cust[pick_id]["x"], id2cust[pick_id]["y"]
        dx, dy = id2cust[deliv_id]["x"], id2cust[deliv_id]["y"]
        mx, my = (px + dx) * 0.5, (py + dy) * 0.5

        dist_with_comp = []
        for comp_idx, dep_id in enumerate(depot_id_list):
            x, y = depot_coords[dep_id]
            d = ((mx - x) ** 2 + (my - y) ** 2) ** 0.5
            dist_with_comp.append((d, comp_idx))
        dist_with_comp.sort(key=lambda t: t[0])
        near = dist_with_comp[0][1]
        second = dist_with_comp[1][1] if len(dist_with_comp) > 1 else dist_with_comp[0][1]
        return near, second
    
    print("\n=== 等距離線付近のタスク交換による改善率平均化 ===")
    previous_cost_reduction_rates = None
    iteration = 0
    step_idx = 2
    while True:
        # --- 終了条件チェック ---
        all_positive = all(rate > 0 for rate in cost_reduction_rates) #個別合理性が満たされている理想的状況
        all_negative = all(rate < 0 for rate in cost_reduction_rates)
        flipped_positive_to_negative = (
            previous_cost_reduction_rates is not None and
            any(prev > 0 and cur < 0 for prev, cur in zip(previous_cost_reduction_rates, cost_reduction_rates))
        )

        if all_positive or all_negative or flipped_positive_to_negative:
            print(">>> 終了条件を満たしたためタスク交換終了")
            break
        
        print(f"--- ラウンド{iteration+1} ---")
        previous_cost_reduction_rates = cost_reduction_rates[:]
        prev_company_costs = current_company_costs
        prev_total_cost = current_total_cost
        
        # 改善率>0の会社を抽出
        improving_companies = [i for i, r in enumerate(cost_reduction_rates) if r > 0]

        # ランキング作成の是非と対象データを決定
        if len(improving_companies) == len(vehicle_num_list):
            break
        else:
            per_company_routes = split_routes_by_company(routes, vehicle_num_list)
            eligible_node_ids = set()
            for comp_idx in improving_companies:
                for r in per_company_routes[comp_idx]:
                    eligible_node_ids.update(r)

            # 対象 customers / PD を絞る
            eligible_customers = [c for c in all_customers if c["id"] in eligible_node_ids]
            eligible_PD_pairs = {p: d for p, d in all_PD_pairs.items()
                                if p in eligible_node_ids and d in eligible_node_ids}

            # ランキングを作成
            ranked = rank_pd_pairs_by_midpoint_to_voronoi_boundary(eligible_customers, eligible_PD_pairs, depot_id_list)


        # 今回の対象 PD を決定
        pick_id, deliv_id = ranked[0][0], ranked[0][1]
        pd_nodes = {pick_id, deliv_id}

        # 現担当会社 & 転送先会社
        current_owner = find_company_owning_pd_pair(routes, vehicle_num_list, pd_nodes)
        nearest_company, second_company = nearest_and_second_depot_company_of_midpoint(pick_id, deliv_id)
        target_owner = second_company

        # 会社ごとに最適化
        per_company_routes = split_routes_by_company(routes, vehicle_num_list)
        new_per_company_routes = []
        for comp_idx, company_routes in enumerate(per_company_routes):
            # 会社内ノード集合・顧客・PD を構築
            sub_customers = filter_subcustomers_by_routes(all_customers, company_routes)
            sub_node_ids = set(c["id"] for c in sub_customers)
            sub_PD_pairs = filter_pd_pairs_for_nodes(all_PD_pairs, sub_node_ids)

            # PD移管ロジック
            if comp_idx == current_owner:
                # 現担当：PD を除外
                sub_customers = [c for c in sub_customers if c["id"] not in pd_nodes]
                sub_node_ids.difference_update(pd_nodes)
                sub_PD_pairs.pop(pick_id, None)  # PD 辞書キーは pickup 側
            elif comp_idx == target_owner:
                # 転送先：PD を追加
                sub_customers.append(id2cust[pick_id])
                sub_customers.append(id2cust[deliv_id])
                sub_node_ids.add(pick_id)
                sub_node_ids.add(deliv_id)
                sub_PD_pairs[pick_id] = deliv_id
            else:
                pass

            # --- 会社ごとに VRP を解く ---
            initial_routes = None
            start_depots = [depot_id_list[comp_idx]] * vehicle_num_list[comp_idx]
            end_depots   = [depot_id_list[comp_idx]] * vehicle_num_list[comp_idx]
            company_route = solve_vrp_flexible(
                sub_customers,
                initial_routes,
                sub_PD_pairs,
                vehicle_num_list[comp_idx],
                vehicle_capacity,
                start_depots,
                end_depots,
                use_capacity=True,
                use_time=True,
                use_pickup_delivery=True,
                isGAT=False
            )
            new_per_company_routes.append(company_route)

        # 全体ルートを連結して更新
        routes = flatten(new_per_company_routes)

        # 改善率の更新
        current_company_costs = compute_company_costs(routes, all_customers, vehicle_num_list)
        current_total_cost = sum(current_company_costs)
        cost_reduction_rates = [((init - cur) / init * 100.0) if init > 0 else 0.0 
                                for init, cur in zip(initial_company_costs, current_company_costs)]
        total_cost_reduction_rates = ((initial_total_cost - current_total_cost) / initial_total_cost * 100.0)
        
        #　[コンソール出力] -> 改善率、他
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
        for idx, (init_c, prev_c, cur_c, init_improve) in enumerate(zip(initial_company_costs, prev_company_costs, current_company_costs, cost_reduction_rates), 1):
            round_improve = ((prev_c - cur_c) / prev_c * 100.0) if prev_c > 0 else 0.0
            print(
                f"LSP {idx:<2} " +
                "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
                    init_c, cur_c, round_improve, init_improve, w=colw
                )
            )
        # TOTAL行
        round_improve_total = ((prev_total_cost - current_total_cost) / prev_total_cost * 100.0) if prev_total_cost > 0 else 0.0
        print(
            f"{'TOTAL':<6} " +
            "{:>{w}.2f} {:>{w}.2f} {:>{w}.2f} {:>{w}.2f}".format(
                initial_total_cost, current_total_cost, round_improve_total, total_cost_reduction_rates, w=colw
            )
        )
        
        # [データ保存] -> jsonファイル、pngファイル
        if ENABLE_EXPORT:
            export_vrp_state(all_customers, routes, all_PD_pairs, step_idx,case_index=case_index,
                         depot_id_list=depot_id_list, vehicle_num_list=vehicle_num_list,instance_name=instance_name, output_root="web_data")
        if ENABLE_PLOT:
            plot_routes(all_customers, routes, depot_id_list, vehicle_num_list,iteration=step_idx, instance_name=instance_name)

        iteration += 1
        step_idx += 1


    #  [データ保存] -> jsonファイル
    if ENABLE_EXPORT:
        generate_index_json(instance_name=instance_name, output_root="web_data", target_root="vrp-viewer/public/vrp_data")

    # 実行時間
    elapsed = time.time() - start_time
    print(f">>> テストケース {case_index} の実行時間: {elapsed:.2f} 秒")
