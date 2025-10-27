import os
import json
import glob
import shutil

def export_vrp_state(customers, routes, PD_pairs, step_index, case_index=None,
                     depot_id_list=None, vehicle_num_list=None, instance_name=None,
                     output_root="web_data"):
    """
    VRP状態をReactアプリ用にJSON形式で保存

    - instance_name が指定されていればそれをディレクトリ名として使う。
      例: instance_name="LC1_2_2_LC1_2_7" -> web_data/LC1_2_2_LC1_2_7/step_0.json
    - instance_name が None の場合は case_{case_index} を使用する。
    - depot_id_list, vehicle_num_list は未指定時に自動推定する。
    """

    
    if instance_name:
        folder_name = instance_name
    else:
        # 保険: case_index が None の場合はタイムスタンプを使う（安全策）
        if case_index is None:
            folder_name = f"case_{int(__import__('time').time())}"
        else:
            folder_name = f"case_{case_index}"

    output_dir = os.path.join(output_root, folder_name)
    # --- 初回のみフォルダをクリーンにする ---
    if step_index == 0:
        if os.path.exists(output_dir):
            print(f"⚠️ 初回ステップのため既存フォルダを削除します: {output_dir}")
            shutil.rmtree(output_dir)
        os.makedirs(output_dir, exist_ok=True)
    else:
        os.makedirs(output_dir, exist_ok=True)

    if depot_id_list is None:
        depot_id_list = [c["id"] for c in customers if c.get("demand", 0) == 0]
    if vehicle_num_list is None:
        vehicle_num_list = [len(routes)]

    data = {
        "customers": customers,
        "routes": routes,
        "PD_pairs": PD_pairs,
        "depot_id_list": depot_id_list,
        "vehicle_num_list": vehicle_num_list,
        "step_index": step_index,
        "instance_name": folder_name
    }

    json_path = os.path.join(output_dir, f"step_{step_index}.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"✅ VRP状態を出力しました: {json_path}")
    return json_path  # 返しておくとテストやログに便利

def generate_index_json(output_root="web_data", target_root="vrp-viewer/public/vrp_data"):
    """
    web_data フォルダ内のすべての instance_name フォルダを走査し、
    target_root にコピーして React 用の index.json を作成する。

    出力形式:
    { "cases": [ {"name": "<instance_name>", "steps": ["step_0.json", ...]}, ... ] }

    - 既に target_root に同名フォルダがあれば上書き（コピー）する。
    """

    os.makedirs(target_root, exist_ok=True)
    cases = []

    for case_dir in sorted(glob.glob(os.path.join(output_root, "*"))):
        if not os.path.isdir(case_dir):
            continue

        case_name = os.path.basename(case_dir)
        steps = sorted([f for f in os.listdir(case_dir) if f.endswith(".json")])
        cases.append({"name": case_name, "steps": steps})

        dest_case_dir = os.path.join(target_root, case_name)
        if os.path.exists(dest_case_dir):
            print(f"⚠️ 既存フォルダを削除します: {dest_case_dir}")
            shutil.rmtree(dest_case_dir)

        shutil.copytree(case_dir, dest_case_dir)
        print(f"📁 コピー完了: {case_dir} → {dest_case_dir}")

    index_path = os.path.join(target_root, "index.json")
    if os.path.exists(index_path):
        print(f"⚠️ 既存の index.json を削除します: {index_path}")
        os.remove(index_path)

    with open(index_path, "w", encoding="utf-8") as f:
        json.dump({"cases": cases}, f, indent=2, ensure_ascii=False)

    print(f"✅ index.json を生成しました → {index_path}")
    return index_path
