import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

  
from collections import namedtuple

Point = namedtuple("Point", ['x', 'y'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def create_data_model(a):
    """Stores the data for the problem."""
    data = {}
    data["locations"] = a
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = int(
                    math.hypot((from_node[0] - to_node[0]), (from_node[1] - to_node[1]))
                )
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    index = routing.Start(0)
    plan_output = "Route:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    #print('003-----',plan_output, route_distance)
    print(f'kuma ya brenda {plan_output} & {route_distance}')
    plan_output += f"Objective: {route_distance}m\n"
    return f'{plan_output},{route_distance} '

def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            routes.append(route)
    
    return routes

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points, points_b = [], []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1])))
        points_b.append((float(parts[0]), float(parts[1])))
        
    data = create_data_model(points_b)
    #print('data',points_b)
    manager = pywrapcp.RoutingIndexManager(
        len(data["locations"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data["locations"])

    #print('001',   distance_matrix)

    
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    #print('002',   transit_callback_index)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    #print('009', solution) weired output
    #sol = ''
    # Print solution on console.
    '''
    if solution:
        #print_solution(manager, routing, solution)
        sol = print_solution(manager, routing, solution)

    print('sol',sol)
    '''
    routes = get_routes(solution, routing, manager)
    solution_ = []
    # Display the routes.
    #print(solution.ObjectiveValue())
    for i, route in enumerate(routes):
        #print('Route', i, route)
        solution_ = route[:-1]
        break

    # build a trivial solution
    # visit the nodes in the order they appear in the file
    obj_0 = length(points[solution_[-1]], points[solution_[0]])
    for index in range(0, nodeCount-1):
        obj_0 += length(points[solution_[index]], points[solution_[index+1]])
    #print(obj_0)
    # calculate the length of the tour
    #obj = solution.ObjectiveValue()
    #print(obj)

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj_0 + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution_))

    return output_data


import sys

if __name__ == '__main__':
    import sys
    file_location = "C:/Users/paul kuria/Documents/tsp/data/tsp_5_1" # tsp_51_1, tsp_5_1
    with open(file_location, 'r') as input_data_file:
        input_data = input_data_file.read()
        print(solve_it(input_data))

    