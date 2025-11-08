# 🚚 Voronoi Routing: Vehicle Routing Optimization via Voronoi Decomposition

このプロジェクトは、**Vehicle Routing Problem (VRP)** を複数の配送会社（LSP）間で分割し、  
**Voronoi 分割 + 社内限定 Genetic Algorithm-based Transportation (GAT)** による経路改善を行う Python 実装。

---

## 🧩 概要

複数の配送会社がそれぞれ顧客を担当する VRP インスタンスを、幾何的な **Voronoi 分割** によって自動的に分割し、  
さらに各会社内部で GAT による経路改善を繰り返し実施する。

全体の流れ：

1. 各社の初期経路を個別に構築  
2. Voronoi 分割により顧客を再配分（配送区域の再定義）  
3. 各社独立で GAT による経路改善を複数ラウンド実施  
4. 各ラウンドでコスト変化と改善率を自動出力・可視化  

---

## 📁 ディレクトリ構造

voronoi_routing/
├── main.py # 実験のメインスクリプト（処理全体を統括）
├── flexible_vrp_solver.py # VRPルートコストや柔軟な評価関数
├── gat.py # 社内GATによるルート改善アルゴリズム
├── voronoi_allocator.py # 顧客のVoronoi分割ロジック
├── visualizer.py # 経路の可視化（matplotlib）
├── web_exporter.py # JSON出力 / Web表示用データ生成
├── parser.py # Li & Lim形式のPDPTWデータパーサ
├── data/ # ベンチマーク入力データ（Li & Lim）
├── figures/ # 各ラウンドで出力されるルート図
└── vrp-viewer/ # Web可視化ツール用データ格納ディレクトリ

## 🧠 アルゴリズム概要

### 🔹 Voronoi Allocation

各LSPのデポを重心点として顧客をVoronoi領域で分割。  
幾何的な近傍分割により、会社間の担当顧客が自動的に再配置される。

**特徴**
- 幾何的な境界を利用することで、直感的かつ高速に顧客を担当会社に割り当て可能  
- 計算コストが低く、大規模インスタンスにも対応  
- 初期解としての品質が高く、その後の局所探索（GAT）が効果的に働く  

---

### 🔹 GAT（Genetic Algorithm-based Transportation）

**基本アイデア**
- 各会社内でのみ遺伝的操作（交換・突然変異など）を許可  
- PDペア（Pick-up / Delivery）単位で交換を行い、制約を常に保持  
- 改善が見られない会社は「収束」とみなし、次ラウンドではスキップ  

**処理の流れ**
1. 各会社のルートを抽出  
2. 会社内の顧客・PDペアを限定してGATを実行  
3. コストが改善された場合のみ次ラウンドに継続  
4. 全社が収束した時点でループ終了  

**終了条件**
- 各社の改善率が `0%`（変化なし）  
- 全ての会社の改善率が `0%` になった時点で全体のGATを終了  

**出力**
- 各ラウンドごとの改善率、会社別コスト、累積改善率を表形式で出力  
- 改善後のルートは PNG と JSON の両方で自動保存（`figures/`, `web_data/`）

---

この2段階構成（Voronoi → GAT）により、  
局所的な最適化と全体的な領域分担の両立を狙う。

