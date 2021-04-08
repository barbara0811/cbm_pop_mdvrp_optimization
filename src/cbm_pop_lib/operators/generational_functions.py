# !/usr/bin/env python

import random
import sys
import os
import rospkg
import networkx as nx
from cbm_pop_lib.common.chromosome import Chromosome
from copy import deepcopy


def init_result(tasks, mdvrp, prec, params):

    result = Chromosome(tasks, mdvrp.max_vehicle_load, prec,
                        mdvrp.sliding_time_windows, mdvrp.n, params)
    for v in range(mdvrp.k):
        result.add_route(v)
    return result


def node_predecessors(node, prec):
    pred = list(prec.predecessors(node))
    for p in prec.predecessors(node):
        pred.extend(node_predecessors(p, prec))
    return list(set(pred))


def node_successors(node, prec):
    succ = list(prec.successors(node))
    for s in prec.successors(node):
        succ.extend(node_successors(s, prec))
    return list(set(succ))


def greedy_insertion(mdvrp, problem_params):
    """Gradually builds the routes by selecting randomly an unserved customer
    and by inserting it at minimum cost in existing routes.

    Returns:
        MDVRP: MDVRP problem instance
    """
    # init prec
    prec = deepcopy(mdvrp.precedence_graph)
    for node in mdvrp.precedence_graph:
        for pred in node_predecessors(node, mdvrp.precedence_graph):
            if (pred, node) not in prec.edges():
                prec.add_edge(pred, node)
        for succ in node_successors(node, mdvrp.precedence_graph):
            if (node, succ) not in prec.edges():
                prec.add_edge(node, succ)

    all_tasks = range(1, mdvrp.n + 1)
    result = init_result(all_tasks, mdvrp, prec, problem_params)

    # all_tasks = deepcopy(temp)
    _constr = list(nx.topological_sort(mdvrp.precedence_graph))
    constr = [x for x in _constr if x in all_tasks]
    ord_tasks = [x for x in all_tasks if x not in constr]
    random.shuffle(ord_tasks)
    ord_tasks = constr + ord_tasks
    check_recursion = 0
    while len(ord_tasks) > 0:
        success = result.insertion_minimal_cost(
            ord_tasks[0], mdvrp.quality_matrix, mdvrp.duration_matrix,
            mdvrp.setup_duration_matrix, mdvrp.demand_matrix,
            mdvrp.setup_cost_matrix)
        if success:
            del ord_tasks[0]
            try:
                c = nx.find_cycle(result.all_constraints)
                print c
                # print self.population[-1].routes
                raw_input("cycle")
            except nx.exception.NetworkXUnfeasible:
                pass
        else:
            x = ord_tasks.pop(0)
            if len(ord_tasks) == 0 or check_recursion > len(all_tasks):
                print result.routes
                print "couldn't do it ........"
                print check_recursion
                raw_input()
                ord_tasks = deepcopy(all_tasks)
                random.shuffle(ord_tasks)
                result = init_result(all_tasks, mdvrp, prec)
                check_recursion = 0
                continue
            ord_tasks.append(x)
            check_recursion += 1
    # raw_input()
    return result
