#!/usr/bin/env python

from cbm_pop_lib.common.mdvrp import MDVRP, Node, Vehicle
from cbm_pop_lib.common.chromosome import Chromosome
from cbm_pop_lib.common import cbm_alg_properties as prop
from math import sqrt, pow
import sys
import os


def calc_route_len(solution, mdvrp):
    duration = 0

    for vehicle in solution.routes.keys():
        route = solution.routes[vehicle][0]
        if len(route) == 0:
            continue
        route_demand = 0
        route_dur = 0
        for i in range(len(route)):
            if i == 0:
                prev_node = 0
            else:
                prev_node = route[i - 1]
            duration += mdvrp.setup_duration_matrix[vehicle][prev_node][route[i]]
            route_dur += mdvrp.setup_duration_matrix[vehicle][prev_node][route[i]]
            route_demand += mdvrp.demand_matrix[vehicle][route[i]]

            if i == len(route) - 1:
                duration += mdvrp.setup_duration_matrix[vehicle][route[i]][0]

    return [duration, route_demand]
    print duration


def load_solution_cordeau(filepath, mdvrp):

    f = open(filepath)
    lines = f.readlines()
    f.close()

    result = Chromosome(mdvrp.precedenceConstraints)
    vehicle_per_depot = mdvrp.k / mdvrp.m

    for v in range(mdvrp.k):
        result.add_route(v)

    for i in range(1, len(lines)):
        data = lines[i].split()

        depot = int(data[0]) - 1
        vehicle = int(data[1]) - 1
        vehicle_id = depot * vehicle_per_depot + vehicle

        for j in range(5, len(data) - 1):
            result.routes[vehicle_id][0].append(int(data[j]))

    calc_route_len(result, mdvrp)
    return result


def save_solution_cordeau(filepath, mdvrp, result, runtime):

    f = open(filepath, "w")
    vehicle_per_depot = mdvrp.k / mdvrp.m

    line = str(result.totalCost) + " " + str(runtime) + "\n"
    f.write(line)
    for vehicle in range(len(result.routes.keys())):
        route = result.routes[vehicle][0]
        if len(route) == 0:
            continue

        line = ""
        depot = vehicle / vehicle_per_depot + 1
        vehicle_ = vehicle % vehicle_per_depot + 1

        line = "{} {}\t{}\t{}\t".format(depot, vehicle_,
                                        result.routeCost[vehicle],
                                        result.capacity[vehicle])
        line += "0 "  # start (depot)
        for node in route:
            line += str(node) + " "
        line += "0\n"  # end (depot)

        f.write(line)

    f.close()


def load_specification_cordeau(filepath):

    f = open(filepath)
    lines = f.readlines()
    f.close()

    data = lines[0].split(' ')
    k = int(data[1])  # vehicle number per depot
    n = int(data[2])  # customer number
    m = int(data[3])  # depot number

    _mdvrp = MDVRP(k, n, m)
    _mdvrp.criteria = prop.problem_criteria.TIME
    _mdvrp.problem_variant = prop.problem_variants.CLASSIC

    # depot info
    depot_index = 0
    for i in range(m + n + 1, 2*m + n + 1):
        data = lines[i].split()

        _mdvrp.depot_vehicles[depot_index] = []

        _mdvrp.depot_labels.append(data[0])
        _mdvrp.nodes[data[0]] = Node(
            float(data[1]), float(data[2]), "depot")

        for j in range(k):
            _mdvrp.depot_vehicles[depot_index].append(depot_index * k + j)

            vehicleLabel = "v" + str(j) + "_" + data[0]
            _mdvrp.vehicle_labels.append(vehicleLabel)
            _mdvrp.vehicles[vehicleLabel] = Vehicle(
                float(data[1]), float(data[2]), _mdvrp.depot_labels[-1])

        depot_index += 1

    # capacity
    d = 0
    for i in range(1, m + 1):
        data = lines[i].split()

        for j in range(k):
            vehicleLabel = "v" + str(j) + "_" + _mdvrp.depot_labels[d]
            if data[1] == "0":
                _mdvrp.vehicles[vehicleLabel].maxLoad = None
                _mdvrp.max_vehicle_load[(i-1)*k + j] = -1
            else:
                _mdvrp.vehicles[vehicleLabel].maxLoad = float(data[1])
                _mdvrp.max_vehicle_load[(i-1)*k + j] = float(data[1])

        d += 1

    # customer info
    for i in range(m + 1, m + n + 1):
        data = lines[i].split()

        _mdvrp.customer_labels.append(data[0])
        _mdvrp.nodes[data[0]] = Node(
            float(data[1]), float(data[2]), "customer")
        duration = float(data[3])
        demand = float(data[4])
        for j in range(_mdvrp.k):
            c = len(_mdvrp.customer_labels)  # customer index
            _mdvrp.duration_matrix[j][c] = duration
            _mdvrp.demand_matrix[j][c] = demand

    calc_setup_duration_matrix(_mdvrp)
    # _mdvrp.draw()
    return _mdvrp


def calc_setup_duration_matrix(_mdvrp):

    for k in range(_mdvrp.m):
        # depot position
        pos_depot = _mdvrp.nodes[_mdvrp.depot_labels[k]].position
        k0 = k * (_mdvrp.k / _mdvrp.m)
        for i in range(_mdvrp.n):
            pos_a = _mdvrp.nodes[_mdvrp.customer_labels[i]].position
            # distance to other customers
            for j in range(i + 1, _mdvrp.n):
                pos_b = _mdvrp.nodes[_mdvrp.customer_labels[j]].position

                dist = sqrt(pow(pos_a[0] - pos_b[0], 2) +
                            pow(pos_a[1] - pos_b[1], 2))

                _mdvrp.setup_duration_matrix[k0, i+1, j+1] = dist
                _mdvrp.setup_duration_matrix[k0, j+1, i+1] = dist
                # setup cost is considered proportionate to the distance
                # _mdvrp.setup_cost_matrix[k0, i+1, j+1] = dist
                # _mdvrp.setup_cost_matrix[k0, j+1, i+1] = dist

            # distance to the depot
            dist = sqrt(pow(pos_a[0] - pos_depot[0], 2) +
                        pow(pos_a[1] - pos_depot[1], 2))
            _mdvrp.setup_duration_matrix[k0, i+1, 0] = dist
            _mdvrp.setup_duration_matrix[k0, 0, i+1] = dist
            # setup cost is considered proportionate to the distance
            # _mdvrp.setup_cost_matrix[k0, i+1, 0] = dist
            # _mdvrp.setup_cost_matrix[k0, 0, i+1] = dist
        for j in range(1, _mdvrp.k / _mdvrp.m):
            _mdvrp.setup_duration_matrix[k0 +
                                         j] = _mdvrp.setup_duration_matrix[k0]
            # _mdvrp.setup_cost_matrix[k0 + j] = _mdvrp.setup_cost_matrix[k0]
