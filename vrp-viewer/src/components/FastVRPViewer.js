import React, { useState, useEffect, useRef } from "react";
import { Stage, Layer, Line, Circle } from "react-konva";

/**
 * FastVRPViewer（全画面キャンバス + y反転 + ズーム + ルート/ノードハイライト + PDペア点線）
 *
 * props:
 * - customers: Array<{ id, x, y, demand, ... }>
 * - routes: number[][]
 * - PD_pairs: { [pickupId: number]: deliveryId: number }
 * - depot_id_list: number[]
 * - vehicle_num_list: number[]
 * - width: number   // 親から与えられる描画領域の幅
 * - height: number  // 親から与えられる描画領域の高さ
 * - onSelectNode?: (info|null) => void  // ノードクリック時に親へ通知（nullで解除）
 */
export default function FastVRPViewer({
  customers,
  routes,
  PD_pairs = {},
  depot_id_list = [],
  vehicle_num_list = [],
  width = 800,
  height = 600,
  onSelectNode,
}) {
  const [selected, setSelected] = useState(null);
  const [highlightedRouteIndex, setHighlightedRouteIndex] = useState(null);
  const stageRef = useRef(null);

  const companyColors = [
    "#007BFF",
    "#28A745",
    "#FFC107",
    "#DC3545",
    "#6F42C1",
    "#20C997",
    "#FF6B6B",
    "#6BCB77",
  ];

  // --- データ辞書 ---
  const idToCoord = Object.fromEntries(
    (customers || []).map((c) => [c.id, { x: Number(c.x), y: Number(c.y) }])
  );
  const idToType = Object.fromEntries(
    (customers || []).map((c) => [
      c.id,
      c.demand > 0 ? "pickup" : c.demand < 0 ? "delivery" : "depot",
    ])
  );

  // --- 経路の所属会社マッピング ---
  const vehicleToCompany = [];
  let vIdx = 0;
  for (let compIdx = 0; compIdx < (vehicle_num_list || []).length; compIdx++) {
    const num = vehicle_num_list[compIdx] || 0;
    for (let k = 0; k < num; k++) {
      vehicleToCompany[vIdx] = compIdx;
      vIdx++;
    }
  }
  for (; vIdx < (routes || []).length; vIdx++) {
    vehicleToCompany[vIdx] = Math.max(0, vehicleToCompany.length - 1);
  }

  // --- 範囲計算 ---
  const allCoords = (customers || []).map((c) => ({
    x: Number(c.x),
    y: Number(c.y),
  }));
  const minX = allCoords.length ? Math.min(...allCoords.map((p) => p.x)) : 0;
  const maxX = allCoords.length ? Math.max(...allCoords.map((p) => p.x)) : 1;
  const minY = allCoords.length ? Math.min(...allCoords.map((p) => p.y)) : 0;
  const maxY = allCoords.length ? Math.max(...allCoords.map((p) => p.y)) : 1;

  // y座標を反転（Konvaキャンバス上で上が+Yになるように）
  const invertY = (y) => maxY - (y - minY);

  // --- ステージのスケール/位置 ---
  const [scale, setScale] = useState(1);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const padding = 40;

  useEffect(() => {
    // キャンバス（親からの実サイズ）にデータ範囲をフィット
    const W = Math.max(1, width - padding * 2);
    const H = Math.max(1, height - padding * 2);
    const dataW = Math.max(1e-6, maxX - minX);
    const dataH = Math.max(1e-6, maxY - minY);
    const newScale = Math.min(W / dataW, H / dataH);
    setScale(newScale);

    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const stageCenterX = width / 2;
    const stageCenterY = height / 2;

    const newPos = {
      x: stageCenterX - centerX * newScale,
      y: stageCenterY - invertY(centerY) * newScale,
    };
    setPosition(newPos);
  }, [customers, width, height, minX, maxX, minY, maxY]);

  // --- ズーム（マウスホイール） ---
  const handleWheel = (e) => {
    e.evt.preventDefault();
    const stage = stageRef.current;
    if (!stage) return;

    const oldScale = stage.scaleX();
    const pointer = stage.getPointerPosition();
    if (!pointer) return;

    const scaleBy = 1.05;
    const direction = e.evt.deltaY > 0 ? 1 / scaleBy : scaleBy;
    const newScale = Math.max(0.1, Math.min(10, oldScale * direction));

    const mousePointTo = {
      x: (pointer.x - stage.x()) / oldScale,
      y: (pointer.y - stage.y()) / oldScale,
    };

    const newPos = {
      x: pointer.x - mousePointTo.x * newScale,
      y: pointer.y - mousePointTo.y * newScale,
    };

    setScale(newScale);
    setPosition(newPos);
    stage.scale({ x: newScale, y: newScale });
    stage.position(newPos);
    stage.batchDraw();
  };

  const idToPoint = (id) => {
    const p = idToCoord[id];
    return { x: p.x, y: invertY(p.y) };
  };

  // --- クリックイベント ---
  const handleBackgroundClick = () => {
    setHighlightedRouteIndex(null);
    setSelected(null);
    if (onSelectNode) onSelectNode(null);
  };

  const handleNodeClick = (nodeId) => {
    const node = idToCoord[nodeId];
    const partner =
      PD_pairs[nodeId] ||
      Object.entries(PD_pairs).find(([p, d]) => Number(d) === nodeId)?.[0];

    const info = {
      id: nodeId,
      x: node.x,
      y: node.y,
      partnerId: partner ? Number(partner) : null,
      partnerCoord: partner ? idToCoord[partner] : null,
      kind: idToType[nodeId],
    };

    setSelected(info);
    if (onSelectNode) onSelectNode(info);

    const idx = routes.findIndex((r) => r.includes(nodeId));
    if (idx !== -1) setHighlightedRouteIndex(idx);
  };

  const handleLineClick = (i) => {
    setHighlightedRouteIndex(i);
    setSelected(null);
    if (onSelectNode) onSelectNode(null);
  };

  // --- 描画 ---
  return (
    <div style={{ width: "100%", height: "100%" }}>
      <Stage
        ref={stageRef}
        width={width}
        height={height}
        draggable
        onWheel={handleWheel}
        scaleX={scale}
        scaleY={scale}
        x={position.x}
        y={position.y}
        onClick={handleBackgroundClick}
        style={{ border: "1px solid #ccc", background: "#fff" }}
      >
        <Layer>
          {/* 経路線 */}
          {routes.map((route, i) => {
            const pts = route.flatMap((id) => {
              const p = idToPoint(id);
              return [p.x, p.y];
            });
            const compIdx = vehicleToCompany[i] || 0;
            const stroke = companyColors[compIdx % companyColors.length];
            const isHighlighted =
              highlightedRouteIndex === null || highlightedRouteIndex === i;

            return (
              <Line
                key={`route_${i}`}
                points={pts}
                stroke={stroke}
                strokeWidth={isHighlighted ? 1.8 / (scale || 1) : 0.6 / (scale || 1)}
                opacity={isHighlighted ? 1 : 0.2}
                lineJoin="round"
                lineCap="round"
                onClick={(e) => {
                  e.cancelBubble = true;
                  handleLineClick(i);
                }}
              />
            );
          })}

          {/* ノード */}
          {customers.map((c) => {
            const p = idToPoint(c.id);
            const fill =
              idToType[c.id] === "pickup"
                ? "#9f4eadff"
                : idToType[c.id] === "delivery"
                ? "#ddb0eae7"
                : "#007BFF";

            const routeIndex = routes.findIndex((r) => r.includes(c.id));
            const isHighlighted =
              highlightedRouteIndex === null || highlightedRouteIndex === routeIndex;

            return (
              <Circle
                key={`node_${c.id}`}
                x={p.x}
                y={p.y}
                radius={6 / (scale || 1)}
                fill={fill}
                opacity={isHighlighted ? 1 : 0.3}
                stroke="#00000022"
                strokeWidth={0.5 / (scale || 1)}
                onClick={(e) => {
                  e.cancelBubble = true;
                  handleNodeClick(c.id);
                }}
              />
            );
          })}

          {/* PDペア点線（選択時のみ） */}
          {selected && selected.partnerCoord && (
            <Line
              points={[
                selected.x,
                invertY(selected.y),
                selected.partnerCoord.x,
                invertY(selected.partnerCoord.y),
              ]}
              stroke="#FF3333"
              strokeWidth={2 / (scale || 1)}
              dash={[1, 0.5]}
              opacity={0.8}
            />
          )}

          {/* デポ枠（円枠） */}
          {depot_id_list.map((dId) => {
            if (!(dId in idToCoord)) return null;
            const p = idToPoint(dId);
            return (
              <Circle
                key={`depot_${dId}`}
                x={p.x}
                y={p.y}
                radius={10 / (scale || 1)}
                fill={null}
                stroke="#000"
                strokeWidth={1 / (scale || 1)}
              />
            );
          })}
        </Layer>
      </Stage>
    </div>
  );
}
