import React from "react";
import FastVRPViewer from "./components/FastVRPViewer";

/**
 * props:
 * - customers
 * - routes
 * - PD_pairs
 * - depot_id_list
 * - vehicle_num_list
 */
export default function InteractiveVRPViewer({
  customers, routes, PD_pairs, depot_id_list, vehicle_num_list,
  width, height, onSelectNode
}) {
  // 受け取った props をそのまま FastVRPViewer に渡す
  if (!customers || !routes) {
    return <div>データがありません</div>;
  }

  return (
    <FastVRPViewer
      customers={customers}
      routes={routes}
      PD_pairs={PD_pairs || {}}
      depot_id_list={depot_id_list || []}
      vehicle_num_list={vehicle_num_list || []}
      width={width}
      height={height}
      onSelectNode={onSelectNode}
    />
  );
}

