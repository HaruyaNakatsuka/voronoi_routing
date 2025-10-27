import matplotlib.pyplot as plt
import numpy as np
import os

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

    # 各実験ごとにフォルダを作成
    instance_folder = os.path.join(output_dir, instance_name)
    os.makedirs(instance_folder, exist_ok=True)

    # ID -> 座標辞書を作成
    id_to_coord = {c["id"]: (c["x"], c["y"]) for c in customers}

    # 各LSPに異なる色を設定
    colors = ["tab:blue", "tab:green", "tab:red", "tab:orange", "tab:purple", "tab:brown"]

    # 描画準備
    plt.figure(figsize=(8, 8))
    plt.title(f"Vehicle Routes (Iteration {iteration})")

    # --- 経路描画 ---
    vehicle_index = 0
    for lsp_index, num_vehicles in enumerate(vehicle_num_list):
        color = colors[lsp_index % len(colors)]
        depot_id = depot_id_list[lsp_index]
        depot_x, depot_y = id_to_coord[depot_id]

        # デポを描画
        plt.scatter(depot_x, depot_y, marker="s", c=color, s=120, edgecolor="black", label=f"LSP {lsp_index+1} depot")

        # 各車両経路を描画
        for _ in range(num_vehicles):
            route = routes[vehicle_index]
            vehicle_index += 1
            if len(route) <= 2:
                continue  # デポのみの車両をスキップ
            xs = [id_to_coord[i][0] for i in route]
            ys = [id_to_coord[i][1] for i in route]
            plt.plot(xs, ys, color=color, alpha=0.8)
            plt.scatter(xs, ys, c=color, s=15)

    # --- 等距離線の描画 ---
    if len(depot_id_list) > 1:
        depot_coords = np.array([id_to_coord[d] for d in depot_id_list])
        x_vals = np.linspace(min(c["x"] for c in customers) - 10, max(c["x"] for c in customers) + 10, 300)
        y_vals = np.linspace(min(c["y"] for c in customers) - 10, max(c["y"] for c in customers) + 10, 300)
        X, Y = np.meshgrid(x_vals, y_vals)

        # 各グリッド点から各デポまでの距離を計算
        distances = np.zeros((len(depot_coords), *X.shape))
        for i, (dx, dy) in enumerate(depot_coords):
            distances[i] = np.sqrt((X - dx)**2 + (Y - dy)**2)

        # デポ間の等距離線を描画
        for i in range(len(depot_coords)):
            for j in range(i + 1, len(depot_coords)):
                plt.contour(X, Y, distances[i] - distances[j], levels=[0], colors="gray", linestyles="--", linewidths=1)

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # ファイル保存
    save_path = os.path.join(instance_folder, f"routes_iter_{iteration:02d}.png")
    plt.savefig(save_path)
    plt.close()
    print(f"✅図を保存しました: {save_path}")

