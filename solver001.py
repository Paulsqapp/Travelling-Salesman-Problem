#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import math
from collections import namedtuple

Point = namedtuple("Point", ['x', 'y'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["locations"] = []
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1])))
        print(parts[0], parts[1])

    print('001', points)

    for x in points:
        print(x[0], x[1])

    # build a trivial solution
    # visit the nodes in the order they appear in the file
    solution = range(0, nodeCount)

    # calculate the length of the tour
    obj = length(points[solution[-1]], points[solution[0]])
    for index in range(0, nodeCount-1):
        obj += length(points[solution[index]], points[solution[index+1]])

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data


import sys

if __name__ == '__main__':
    import sys
    file_location = "D:/discrete opt/3 local search/_kK3nKenTFSCt5ynp_xUWg_563879a7fb4b4b6a9efa9781a2398d6a_tsp/tsp/data/tsp_5_1"
    with open(file_location, 'r') as input_data_file:
        input_data = input_data_file.read()
        print(solve_it(input_data))
    