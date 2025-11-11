import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
import json
import math
import logging

logger = logging.getLogger(__name__)

plt.rcParams['font.family'] = 'MS Gothic'  # Windowsの場合
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.monospace'] = ['MS Gothic']

def plot_routes(customers, routes, depot_id_list, vehicle_num_list, iteration, instance_name="", output_dir="figures"):
    """
    各車両の経路を描画し保存する関数（等距離線付き）
    - customers: 全顧客データ
    - routes: 2次元リスト、全車両の経路
    - depot_id_list: 各社のデポid
    - vehicle_num_list: 各社の車両数
    - iteration: 現在の反復番号（ファイル名に使用）
    - instance_name: 実験インスタンス名（フォルダ作成用）
    """

    # ====== 内部ユーティリティ ======
    def route_cost(single_route, id2xy):
        cost = 0.0
        for k in range(len(single_route) - 1):
            x1, y1 = id2xy[single_route[k]]
            x2, y2 = id2xy[single_route[k + 1]]
            cost += math.hypot(x2 - x1, y2 - y1)
        return cost

    def company_costs(all_routes, id2xy, veh_num_list):
        costs = []
        idx = 0
        for n in veh_num_list:
            s = 0.0
            for _ in range(n):
                s += route_cost(all_routes[idx], id2xy)
                idx += 1
            costs.append(s)
        return costs, sum(costs)

    def load_step_routes(step_index):
        base = os.path.join("web_data", instance_name, f"step_{step_index}.json")
        if not os.path.isfile(base):
            return None
        try:
            with open(base, "r", encoding="utf-8") as f:
                j = json.load(f)
            id2 = {c["id"]: (c["x"], c["y"]) for c in j["customers"]}
            comp_costs, total = company_costs(j["routes"], id2, j.get("vehicle_num_list", vehicle_num_list))
            return {"company": comp_costs, "total": total}
        except Exception:
            return None

    def fmt(v):
        return f"{v:.2f}"

    def pct(old, new):
        if old and old > 0:
            return f"{(old - new) / old * 100:.2f}%"
        return "—"

    # ====== フォルダ準備（初回のみ全消去） ======
    instance_folder = os.path.join(output_dir, instance_name)
    if iteration == 0 and os.path.isdir(instance_folder):
        shutil.rmtree(instance_folder)
    os.makedirs(instance_folder, exist_ok=True)

    # ID -> 座標辞書
    id_to_coord = {c["id"]: (c["x"], c["y"]) for c in customers}

    # 色
    colors = ["tab:blue", "tab:green", "tab:red", "tab:orange", "tab:purple", "tab:brown"]

    # ====== 図のセットアップ ======
    plt.figure(figsize=(8, 8))
    if iteration == 0:
        plt.title(f"{instance_name}：初期解")
    elif iteration == 1:
        plt.title(f"{instance_name}：ボロノイ分割後")
    else:
        plt.title(f"{instance_name} ：Iteration {iteration-1}")

    # 経路描画
    vehicle_index = 0
    for lsp_index, num_vehicles in enumerate(vehicle_num_list):
        color = colors[lsp_index % len(colors)]
        depot_id = depot_id_list[lsp_index]
        depot_x, depot_y = id_to_coord[depot_id]
        plt.scatter(depot_x, depot_y, marker="s", c=color, s=120, edgecolor="black", label=f"LSP {lsp_index+1} depot")

        for _ in range(num_vehicles):
            route = routes[vehicle_index]
            vehicle_index += 1
            if len(route) <= 2:
                continue
            xs = [id_to_coord[i][0] for i in route]
            ys = [id_to_coord[i][1] for i in route]
            plt.plot(xs, ys, color=color, alpha=0.8)
            plt.scatter(xs, ys, c=color, s=15)

    # 等距離線
    if len(depot_id_list) > 1:
        depot_coords = np.array([id_to_coord[d] for d in depot_id_list])
        x_vals = np.linspace(min(c["x"] for c in customers) - 10, max(c["x"] for c in customers) + 10, 300)
        y_vals = np.linspace(min(c["y"] for c in customers) - 10, max(c["y"] for c in customers) + 10, 300)
        X, Y = np.meshgrid(x_vals, y_vals)
        distances = np.zeros((len(depot_coords), *X.shape))
        for i, (dx, dy) in enumerate(depot_coords):
            distances[i] = np.sqrt((X - dx)**2 + (Y - dy)**2)
        for i in range(len(depot_coords)):
            for j in range(i + 1, len(depot_coords)):
                plt.contour(X, Y, distances[i] - distances[j], levels=[0], colors="gray", linestyles="--", linewidths=1)

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # ====== メトリクス計算 ======
    curr_company, curr_total = company_costs(routes, id_to_coord, vehicle_num_list)

    lines = []
    if iteration == 0:
        lines.append("【初期解】")
        for i, c in enumerate(curr_company, 1):
            lines.append(f"  LSP {i}: {fmt(c)}")
        lines.append(f"  TOTAL: {fmt(curr_total)}")
    elif iteration == 1:
        lines.append("【ボロノイ分割後】")
        init_metrics = load_step_routes(0)
        for i, c in enumerate(curr_company, 1):
            base = init_metrics["company"][i-1] if init_metrics else None
            lines.append(f"  LSP {i}: {fmt(c)}   改善(初期比): {pct(base, c)}")
        base_total = init_metrics["total"] if init_metrics else None
        lines.append(f"  TOTAL: {fmt(curr_total)}   改善(初期比): {pct(base_total, curr_total)}")
    else:
        lines.append(f"【等距離線付近のタスク交換{iteration-1}回目】")
        init_metrics = load_step_routes(0)
        voro_metrics = load_step_routes(1)
        prev_metrics = load_step_routes(iteration-1)
        for i, c in enumerate(curr_company, 1):
            base_prev = prev_metrics["company"][i-1] if prev_metrics else None
            base_voro = voro_metrics["company"][i-1] if voro_metrics else None
            base_init = init_metrics["company"][i-1] if init_metrics else None
            lines.append(f"    LSP {i}: {fmt(c)}   改善(ラウンド比): {pct(base_prev, c)}   改善(ボロノイ比): {pct(base_voro, c)}   改善(初期比): {pct(base_init, c)}")
        base_total_prev = prev_metrics["total"] if prev_metrics else None
        base_total_voro = voro_metrics["total"] if voro_metrics else None
        base_total_init = init_metrics["total"] if init_metrics else None
        lines.append(f"    TOTAL: {fmt(curr_total)}   改善(ラウンド比): {pct(base_total_prev, curr_total)}   改善(ボロノイ比): {pct(base_total_voro, curr_total)}   改善(初期比): {pct(base_total_init, curr_total)}")

    # ====== ★ここを変更：下部フッターに表示 ======
    if lines:
        text = "\n".join(lines)
        fig = plt.gcf()
        # 下に十分な余白（フッター）を確保（数値は必要に応じて微調整）
        plt.subplots_adjust(bottom=0.17)
        # 図座標（0～1）で左下寄せに描画
        fig.text(
            0.02, 0.02, text,
            ha="left", va="bottom",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9, edgecolor="#999"),
            family="monospace"
        )

    # 保存
    save_path = os.path.join(instance_folder, f"routes_iter_{iteration:02d}.png")
    plt.savefig(save_path)
    plt.close()
    logger.info(f"✅図を保存しました: {save_path}")
