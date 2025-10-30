from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

def create_distance_matrix(customers):
    size = len(customers)
    matrix = [[0] * size for _ in range(size)]
    for i in range(size):
        for j in range(size):
            matrix[i][j] = int(math.hypot(customers[i]['x'] - customers[j]['x'], customers[i]['y'] - customers[j]['y']))
    return matrix


def solve_vrp_flexible(customers, initial_routes, PD_pairs, num_vehicles, vehicle_capacity, start_depots, end_depots,
                       use_capacity:bool, use_time:bool, use_pickup_delivery:bool, isGAT:bool):
    # 距離行列を作成
    distance_matrix = create_distance_matrix(customers)
    
     # 顧客ID → インデックス変換辞書
    id_to_index = {c['id']: i for i, c in enumerate(customers)}
    # 各車両のデポidをインデックスに変換（RoutingIndexManagerに渡す形式）
    starts = [id_to_index[depot_id] for depot_id in start_depots]
    ends = [id_to_index[depot_id] for depot_id in end_depots]

    # routing index managerを作成
    manager = pywrapcp.RoutingIndexManager(len(customers), num_vehicles, starts, ends)
    # Routing Modelを作成
    routing = pywrapcp.RoutingModel(manager)

    # transit callbackを作成・登録
    def distance_callback(from_idx, to_idx):
        from_node = manager.IndexToNode(from_idx)
        to_node = manager.IndexToNode(to_idx)
        return distance_matrix[from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    #各アークのコストを定義（コスト＝距離）
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # 容量制約
    if use_capacity:
        demands = [c['demand'] for c in customers]
        def demand_callback(from_idx):
            return demands[manager.IndexToNode(from_idx)]
        demand_cb = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_cb, 0, [vehicle_capacity] * num_vehicles, True, 'Capacity'
        )

    # 時間制約
    if use_time:
        time_windows = [(c['ready'], c['due']) for c in customers]
        service_times = [c['service'] for c in customers]
        def time_callback(from_idx, to_idx):
            from_node = manager.IndexToNode(from_idx)
            to_node = manager.IndexToNode(to_idx)
            return distance_matrix[from_node][to_node] + service_times[from_node]
        time_cb = routing.RegisterTransitCallback(time_callback)
        routing.AddDimension(time_cb, 99999, 99999, False, "Time")
        time_dim = routing.GetDimensionOrDie("Time")
        for node_idx in range(len(customers)):
            idx = manager.NodeToIndex(node_idx)
            time_dim.CumulVar(idx).SetRange(*time_windows[node_idx])

    # Pickup and Delivery 制約
    if use_pickup_delivery:
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            10000,  # vehicle maximum travel distance
            True,  # start cumul to zero
            "Distance",
        )
        distance_dimension = routing.GetDimensionOrDie("Distance")
        distance_dimension.SetGlobalSpanCostCoefficient(100)
        
        for pickup_id, delivery_id in PD_pairs:
            if pickup_id not in id_to_index or delivery_id not in id_to_index:
                print(f"Invalid ID pair: {pickup_id}, {delivery_id}")
                continue
            pickup_idx = manager.NodeToIndex(id_to_index[pickup_id])
            delivery_idx = manager.NodeToIndex(id_to_index[delivery_id])

            routing.AddPickupAndDelivery(pickup_idx, delivery_idx)
            routing.solver().Add(routing.VehicleVar(pickup_idx)
                                 == routing.VehicleVar(delivery_idx))
            routing.solver().Add(distance_dimension.CumulVar(pickup_idx)
                                 <= distance_dimension.CumulVar(delivery_idx))

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    #search_params.log_search = True

    if isGAT:
        #search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
        search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC

        # idをローカルインデックスに変換
        initial_routes_local = []
        for route in initial_routes:
            initial_routes_local.append([id_to_index[node_id] for node_id in route])

        routing.CloseModelWithParameters(search_params)
        initial_solution = routing.ReadAssignmentFromRoutes(initial_routes_local, True)
        solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_params)
    else:
        search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
        search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC
        solution = routing.SolveWithParameters(search_params)
    
    if not solution:
        print("No solution found.")
        return None
    

    
    # 解の取得
    result = []
    for vehicle_id in range(num_vehicles):
        idx = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(idx):
            route.append(customers[manager.IndexToNode(idx)]['id'])
            idx = solution.Value(routing.NextVar(idx))
        route.append(customers[manager.IndexToNode(idx)]['id'])
        result.append(route)

    return result

def route_cost(route, customers):
    """ルートの総距離を計算する簡易関数"""
    id_to_coord = {c['id']: (c['x'], c['y']) for c in customers}
    cost = 0
    for i in range(len(route) - 1):
        x1, y1 = id_to_coord[route[i]]
        x2, y2 = id_to_coord[route[i + 1]]
        cost += ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    return cost