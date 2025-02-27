#!/usr/bin/python
# -*- coding: utf-8 -*-

# multiply by 100

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import numpy as np


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()} miles")
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    print(plan_output)
    plan_output += f"Route distance: {route_distance}miles\n"

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

    points = dict()
    points['distance_matrix'] = []
    points["num_vehicles"] = 1
    points["depot"] = 0
    all_point = []
    for i in range(1, nodeCount+1):
        #print('i',i)
        line = lines[i]
        parts = line.split()
        
        #print('start_point',start_point)
        current_point = np.array([float(parts[0]), float(parts[1])])
        all_point.append(current_point)
        
        
    #print('001', all_point)

    for x in all_point:
        distance = [int(np.linalg.norm(x - b)*1000) for b in all_point]
        points['distance_matrix'].append(distance)
        
    #print('distance \n', points['distance_matrix'])

    manager = pywrapcp.RoutingIndexManager(
        len(points["distance_matrix"]), points["num_vehicles"], points["depot"]
        )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return points['distance_matrix'][from_node][to_node]
    
    #print('routing', routing)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
     # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION 
    ) #FIRST_UNBOUND_MIN_VALUE, SWEEP, SAVINGS #AUTOMATIC, PATH_CHEAPEST_ARC, CHRISTOFIDES
    '''
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING) #AUTOMATIC, TABU_SEARCH, GENERIC_TABU_SEARCH, SIMULATED_ANNEALING
    search_parameters.time_limit.seconds = 30
    #search_parameters.log_search = True
    '''
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    '''
    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)
        return True
    #else:
    #    return False
    '''
    routes = get_routes(solution, routing, manager)
    # Display the routes.
    
    cost = round((solution.ObjectiveValue()/1000), 2)
    #print('cost',str(routes[0][:-1]))
    
    #print(routes, 'distance', solution.ObjectiveValue()/10, )
    #dist_2 = np.sum(points['distance_matrix'])
    #print('sum',dist_2/10)

    # build a trivial solution
    # visit the nodes in the order they appear in the file
    
    # prepare the solution in the specified output format
    output_data = str(cost)+' '  + str(0) + '\n'
    output_data += ' '.join(map(str, routes[0][:-1]))

    return output_data



if __name__ == '__main__':
    import sys
    file_location = "data/tsp_51_1" # data\tsp_51_1, data/tsp_5_1 tsp_1400_1 data/tsp_100_1
    with open(file_location, 'r') as input_data_file:
        input_data = input_data_file.read()
        print(solve_it(input_data))
    
