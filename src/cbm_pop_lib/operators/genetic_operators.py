#!/usr/bin/env python

import random
from copy import deepcopy
import sys
import os
import cbm_pop_lib.operators.auxiliary_functions as aux
from cbm_pop_lib.common import cbm_alg_properties as prop
import networkx as nx


# Crossover

def crossover_bcrc(vehicle_candidates, p1, p2, quality, duration,
                   setup_time, demand, setup_cost):
    """Best cost route crossover.

    Args:
        depot (int): depot id
        depot_vehicles (dict): key= depot ids, value= list of vehicles
        p1 (Chromosome): first parent
        p2 (Chromosome): second parent
        quality (np.matrix): quality matrix for all vehicles
        duration (np.matrix): duration matrix for all vehicles
        setup_time (np.matrix): setup time matrix for all vehicles
        demand (np.matrix): demand matrix for all vehicles
        setup_cost (np.matrix): setup cost matrix for all vehicles

    Returns:
        Chromosome list: two generated offsprings
    """
    # offsprings
    o1 = p1.clone()
    o2 = p2.clone()

    # select a random route from p1 and p2
    v1 = random.choice(vehicle_candidates)
    r1 = deepcopy(random.choice(p1.routes[v1]))

    v2 = random.choice(vehicle_candidates)
    r2 = deepcopy(random.choice(p2.routes[v2]))

    diff1 = set(p1.all_customers) - set(p2.all_customers)
    diff2 = set(p2.all_customers) - set(p1.all_customers)
    # check for decomposition compatibility
    if len(diff1) + len(diff2) > 0:
        print "not compatible"
        return [p1, p2]

    # remove nodes in r2 from o1 and nodes in r1 from o2
    [succ, _] = o1.remove_nodes(r2, quality, duration, setup_time, demand, setup_cost)
    if not succ:
        #print "can't remove"
        return [p1, p2]
    [succ, _] = o2.remove_nodes(r1, quality, duration, setup_time, demand, setup_cost) 
    if not succ:
        #print "can't remove"
        return [p1, p2]

    # insert nodes from r2 into routes of o1 (with best insertion cost)
    _constr = list(nx.topological_sort(o1.prec_constraints))
    r2_c = [x for x in _constr if x in r2]
    r2_nc = [x for x in r2 if x not in r2_c]
    random.shuffle(r2_nc)
    r2_ord = r2_c + r2_nc
    rec = len(r2_ord) * 2
    while len(r2_ord) > 0:
        if not o1.insertion_minimal_cost(r2_ord[0], quality, duration, setup_time, demand,
                                  setup_cost):
            r2_ord.append(r2_ord[0])
        del r2_ord[0]
        rec -= 1
        if rec == 0:
            print "can't insert"
            return [p1, p2]

    # insert nodes from r1 into routes of o2 (with best insertion cost)
    _constr = list(nx.topological_sort(o2.prec_constraints))
    r1_c = [x for x in _constr if x in r1]
    r1_nc = [x for x in r1 if x not in r1_c]
    random.shuffle(r1_nc)
    r1_ord = r1_c + r1_nc
    rec = len(r1_ord) * 2
    while len(r1_ord) > 0:
        if not o2.insertion_minimal_cost(r1_ord[0], quality, duration, setup_time, demand,
                                  setup_cost):
            r1_ord.append(r1_ord[0])
        del r1_ord[0]
        rec -= 1
        if rec == 0:
            print "can't insert"
            return [p1, p2]

    print "ok"
    return [o1, o2]


# Mutation

def reverse_nodes(p, vehicle, cut1, cut2, duration, setup_time, setup_cost):
    """Reverse nodes between two setpoints.

    Args:
        p (Chromosome): parent
        vehicle (int): vehicle id
        cut1 (int): first cut index (enters reversion)
        cut2 (int): second cut index (excluded for the reversion)
        duration (np.matrix): duration matrix for all vehicles
        setup_time (np.matrix): setup time matrix for all vehicles
        setup_cost (np.matrix): setup cost matrix for all vehicles

    Returns:
        Chromosome: generated offspring
    """
    p_res = p.clone()
    customers = []
    ind = 0
    cuts = [[-1, -1], [-1, -1]]
    for i in range(len(p.routes[vehicle])):
        if (cut1 >= ind and cut1 < ind + len(p.routes[vehicle][i]) and
                cuts[0][0] == -1):
            cuts[0][0] = i
            cuts[0][1] = cut1 - ind
        if (cut2 >= ind and cut2 < ind + len(p.routes[vehicle][i]) + 1 and
                cuts[1][0] == -1):
            cuts[1][0] = i
            cuts[1][1] = cut2 - ind
        ind += len(p.routes[vehicle][i])
        if cuts[0][0] == i:
            if cuts[1][0] != i:
                customers.extend(p.routes[vehicle][i][cuts[0][1]:])
            else:
                customers.extend(p.routes[vehicle][i][cuts[0][1]:cuts[1][1]])
        elif cuts[1][0] == i:
            customers.extend(p.routes[vehicle][i][:cuts[1][1]])

    customers.reverse()
    constrained = p.prec_constraints.nodes()
    customers = [x for x in customers if x not in constrained]

    cost_diff = 0
    ind = 0
    i = cuts[0][0]
    while i <= cuts[1][0]:
        if i == cuts[0][0]:
            j = cuts[0][1]
        else:
            j = 0
        if i == cuts[1][0]:
            upper_limit = cuts[1][1]
        else:
            upper_limit = len(p_res.routes[vehicle][i])
        old_route = deepcopy(p_res.routes[vehicle][i])
        while j < upper_limit:
            old_node = p_res.routes[vehicle][i][j]
            if old_node in constrained:
                j += 1
                continue
            new_node = customers[ind]
            # prev node
            if j == 0:
                old_prev_node = 0
                prev_node = 0
            else:
                old_prev_node = old_route[j-1]
                prev_node = p_res.routes[vehicle][i][j-1]
            # bond with previous node
            cost_diff += (setup_cost[vehicle][prev_node][new_node] -
                          setup_cost[vehicle][old_prev_node][old_node])
            # check if the procedure is finishing
            if j == upper_limit - 1:
                if upper_limit < len(p_res.routes[vehicle][i]):
                    next_node = p_res.routes[vehicle][i][j+1]
                else:
                    next_node = 0
                # if classic vrp --> count return to depot
                if (next_node > 0) or (p_res.problem_variant == prop.problem_variants.CLASSIC):
                    cost_diff += (setup_cost[vehicle][new_node][next_node] -
                                  setup_cost[vehicle][old_node][next_node])
            p_res.routes[vehicle][i][j] = new_node
            ind += 1
            j += 1
        i += 1

    # update cost
    p_res.total_cost += cost_diff
    p_res.capacity[vehicle] -= cost_diff
    success = 1
    if p_res.capacity[vehicle] <= 0:
        success = 0

    return [success, p_res]


swap_succ = 0
swap_fail = 0


def swap_nodes(p, v1, r1, n1_index, v2, r2, n2_index, quality, duration,
               setup_time, demand, setup_cost):
    """Swap two nodes.

    Args:
        p (Chromosome): parent chromosome
        v1 (int): first vehicle id
        r1 (int): first route index
        n1_index (int): first node index
        v2 (int): second vehicle id
        r2 (int): second route index
        n2_index (int): second node index
        quality (np.matrix): quality matrix for all vehicles
        duration (np.matrix): duration matrix for all vehicles
        setup_time (np.matrix): setup time matrix for all vehicles
        demand (np.matrix): demand matrix for all vehicles
        setup_cost (np.matrix): setup cost matrix for all vehicles

    Returns:
        Chromosome: generated offspring
    """
    route1 = p.routes[v1][r1]
    node1 = route1[n1_index]
    route2 = p.routes[v2][r2]
    node2 = route2[n2_index]

    p_res = p.clone()
    old_cost = p.total_cost
    # remove node1 from (v1,r1) and put node2 in the spot
    # remove node2 from (v2,r2) and put node1 in the spot
    if not p_res.remove_node(v1, r1, n1_index, quality[v1], duration, setup_time,
                      demand[v1], setup_cost[v1]):
        return [False, p, 0]
    if not p_res.remove_node(v2, r2, n2_index, quality[v2], duration, setup_time,
                      demand[v2], setup_cost[v2]):
        return [False, p, 0]
    # insert node2 in route1 in n1_index
    [a, b] = p_res.calc_possible_insertions(route1, node2)
    if n1_index >= a and n1_index < b:
        if not p_res.add_node(node2, v1, r1, n1_index, quality[v1], duration,
                              setup_time, demand[v1], setup_cost[v1]):
            return [False, p, 0]
    else:
        return [False, p, 0]
    # insert node1 in route2 in n1_index
    [a, b] = p_res.calc_possible_insertions(route2, node1)
    if n2_index >= a and n2_index < b:
        if not p_res.add_node(node1, v2, r2, n2_index, quality[v2], duration,
                              setup_time, demand[v2], setup_cost[v2]):
            return [False, p, 0]
    else:
        return [False, p, 0]
    return [True, p_res, p_res.total_cost - old_cost]


def mutation_intra_depot_reversal(p1, vehicle, duration, setup_time,
                                  setup_cost):
    """Intra depot reversal mutation. Two cutpoints are selected in the
    chromosome associated with the depot, and the genetic material between
    these two cutpoints is reversed.

    Args:
        p1 (Chromosome): parent chromosome
        vehicle (int): vehicle id
        duration (np.matrix): duration matrix for all vehicles
        setup_time (np.matrix): setup time matrix for all vehicles
        setup_cost (np.matrix): setup cost matrix for all vehicles

    Returns:
        Chromosome: generated offspring
    """
    customers = []
    for r in p1.routes[vehicle]:
        customers.extend(r)

    if len(customers) == 0:
        return p1
    # cut1 -- first element that gets into inversion list
    # cut2 -- first element after inversion list that doesn't get into
    #         inversion list
    cut1 = random.randint(0, len(customers) - 1)
    if cut1 == len(customers) - 1:
        cut2 = len(customers)
    else:
        cut2 = random.randint(cut1 + 1, len(customers))

    [success, p_res] = reverse_nodes(p1, vehicle, cut1, cut2, duration, setup_time,
                                     setup_cost)
    if not success:
        return p1
    return p_res


def mutation_intra_depot_swapping(p1, depot, depot_vehicles, quality, duration,
                                  setup_time, demand, setup_cost):
    """This simple mutation operator selects two random routes and swaps one
    randomly chosen customer from one route to another.

    Args:
        p1 (Chromosome): parent chromosome
        depot (int): depot id
        depot_vehicles (int list): a list of vehicles at the depot
        quality (np.matrix): quality matrix for all vehicles
        duration (np.matrix): duration matrix for all vehicles
        setup_time (np.matrix): setup time matrix for all vehicles
        demand (np.matrix): demand matrix for all vehicles
        setup_cost (np.matrix): setup cost matrix for all vehicles

    Returns:
        Chromosome: generated offspring
    """
    # list feasible vehicles
    vehicle_candidates = []
    for v in depot_vehicles:
        if len(p1.routes[v]) == 1:
            if len(p1.routes[v][0]) == 0:
                continue
        vehicle_candidates.append(v)

    # select two vehicles
    if len(vehicle_candidates) < 2:
        return p1

    v1 = random.choice(vehicle_candidates)
    r1 = random.randint(0, len(p1.routes[v1]) - 1)
    v2 = random.choice(vehicle_candidates)
    r2 = random.randint(0, len(p1.routes[v2]) - 1)
    while v1 == v2 and r1 == r2:
        v2 = random.choice(vehicle_candidates)
        r2 = random.randint(0, len(p1.routes[v2]) - 1)

    n1_index = random.randint(0, len(p1.routes[v1][r1]) - 1)
    n2_index = random.randint(0, len(p1.routes[v2][r2]) - 1)

    [success, p_res, cost] = swap_nodes(p1, v1, r1, n1_index, v2, r2, n2_index,
                                        quality, duration, setup_time, demand,
                                        setup_cost)
    return p_res


def get_swap_candidates(p1, d1, v1, n1, borderline_customers,
                        candidate_depots, depot_vehicles, demand):
    """Get candidate nodes for inter route swap with the node1 currently
    assigned to the vehicle v1 from the depot d1.

    Args:
        p1 (Chromosome): parent chromosome
        d1 (int): depot id
        v1 (int): vehicle id
        node1 (int): node1 id
        borderline_customers (int list): candidate nodes for swap
        candidate_depots (dict): key= node id, value= a list of possible depots
        depot_vehicles (dict): key= depot id, value= a list of vehicles at the depot
        demand (np.matrix): demand matrix for all vehicles

    Returns:
        two lists: [description]
    """
    candidates = []
    candidate_vehicles = []
    for n2 in borderline_customers:
        if len(set(candidate_depots[n1]) & set(candidate_depots[n2])) > 1:
            # find a route where this node is
            for depot in candidate_depots[n2]:
                if depot == d1:
                    break  # this is no longer a candidate (same depot node)
                for v2 in depot_vehicles[depot]:
                    if n2 in p1.routes[v2][0]:
                        candidates.append(n2)
                        candidate_vehicles.append(v2)
    return [candidates, candidate_vehicles]


def mutation_inter_depot_swapping(p1, depot_vehicles, borderline_customers,
                                  candidate_depots, quality, duration,
                                  setup_time, demand, setup_cost):
    """
    """
    if len(borderline_customers) == 0:
        return p1

    # select node 1
    node1 = random.choice(borderline_customers)
    while node1 in p1.unserved_customers:
        node1 = random.choice(borderline_customers)
    [d1, v1, r1, n1_index] = aux.find_node_route(p1, node1,
                                                 len(depot_vehicles[0]))
    if min(d1, v1, r1, n1_index) < 0:
        return p1
    [swap_candidates, swap_candidate_vehicles] = get_swap_candidates(
        p1, d1, v1, node1, borderline_customers, candidate_depots,
        depot_vehicles, demand)

    # select node 2
    if len(swap_candidates) > 0:
        n2_choice = random.randint(0, len(swap_candidates) - 1)
        node2 = swap_candidates[n2_choice]
        v2 = swap_candidate_vehicles[n2_choice]
        r2 = 0
        n2_index = p1.routes[v2][r2].index(node2)
    else:
        return p1

    [success, p_res, cost] = swap_nodes(p1, v1, r1, n1_index, v2, r2, n2_index,
                                        quality, duration, setup_time, demand,
                                        setup_cost)
    return p_res


def mutation_single_customer_rerouting(vehicle, p1, quality, duration,
                                       setup_time, setup_cost, demand):
    """Re-routing involves randomly selecting one customer, and removing that
    customer from the existing route. The customer is then inserted in the
    best feasible insertion location within the entire chromosome.
    """
    route = random.randint(0, len(p1.routes[vehicle]) - 1)
    if len(p1.routes[vehicle][route]) < 1:
        return p1
    removal_index = random.randint(0, len(p1.routes[vehicle][route]) - 1)
    node = p1.routes[vehicle][route][removal_index]

    p_res = p1.clone()
    # remove customer from chromosome
    if not p_res.remove_node(vehicle, route, removal_index, quality[vehicle],
                      duration, setup_time, demand[vehicle],
                      setup_cost[vehicle]):
        return p1

    # insert customer at best insert location
    if p_res.insertion_minimal_cost(node, quality, duration, setup_time,
                                    demand, setup_cost):
        return p_res
    return p1


############################
# Local search algorithms  #
############################

def two_opt(p1, duration, setup_time, setup_cost):

    for v in p1.routes.keys():
        for r in range(len(p1.routes[v])):
            two_opt_route(p1, v, r, duration, setup_time, setup_cost)
    return p1


def two_opt_route(p1, vehicle, route, duration, setup_time, setup_cost):

    r = p1.routes[vehicle][route]
    nochange = False
    maxIter = 50

    while not nochange:
        maxIter -= 1
        if maxIter == 0:
            break

        nochange = True
        for i in range(0, len(r) - 3):
            a = r[i]
            b = r[i + 1]
            for j in range(i + 2, len(r) - 1):
                c = r[j]
                d = r[j + 1]
                lenChange = (setup_cost[vehicle][a][c] + setup_cost[vehicle][b][d] -
                             setup_cost[vehicle][a][b] - setup_cost[vehicle][c][d])

                if lenChange < 0:
                    # reverse
                    [success, p_tmp] = reverse_nodes(p1, vehicle, i + 1, j + 1,
                                                     duration, setup_time, setup_cost)
                    if success:
                        p1 = p_tmp.clone()
                        nochange = False


def one_move(p1, quality, duration, setup_time, demand, setup_cost):
    time_criteria = [prop.problem_criteria.TIME,
                     prop.problem_criteria.MAKESPAN,
                     prop.problem_criteria.MAKESPANCOST]
    cost_criteria = [prop.problem_criteria.COST,
                     prop.problem_criteria.COSTMAKESPAN]
    for vehicle in p1.routes.keys():
        for r in range(len(p1.routes[vehicle])):
            removal_index = 0
            while removal_index < len(p1.routes[vehicle][r]):
                p_tmp = p1.clone()
                node = p1.routes[vehicle][r][removal_index]
                # remove customer from chromosome
                if not p_tmp.remove_node(vehicle, r, removal_index, quality[vehicle],
                                  duration, setup_time, demand[vehicle],
                                  setup_cost[vehicle]):
                    continue
                # insert customer at best insert location
                p_tmp.insertion_minimal_cost(
                    node, quality, duration, setup_time, demand,
                    setup_cost)
                if p1.criteria in time_criteria:
                    p_tmp.evaluate_schedule(duration, setup_time)
                    imp = p_tmp.total_duration - p1.total_duration
                elif p1.criteria in cost_criteria:
                    imp = p_tmp.total_cost - p1.total_cost
                if imp:
                    p1 = p_tmp.clone()

                removal_index += 1
    return p1


def calc_insertion_reduce_idle(p1, v, r, candidate, i, limit_low, limit_up,
                               idle_slot, duration, setup_time, setup_cost,
                               demand):
    # remove node from its place
    cost_diff = 0
    p = p1.routes[v][r][i-1]
    cost_diff -= setup_cost[v][p][candidate]
    if i < len(p1.routes[v][r]) - 1:
        n = p1.routes[v][r][i+1]
        cost_diff += (setup_cost[v][p][n] - setup_cost[v][candidate][n])

    best_ins = [-1, -1, -1]
    # find insertion place
    for j in range(limit_low, limit_up + 1):
        p = 0
        if j > 0:
            p = p1.routes[v][r][j-1]
        n = p1.routes[v][r][j+1]
        dur_diff_ins = (setup_time[v][p][candidate] + duration[v][candidate] +
                        setup_time[v][candidate][n] - setup_time[v][p][n])
        cost_diff_ins = (setup_cost[v][p][candidate] +
                         setup_cost[v][candidate][n] -
                         setup_cost[v][p][n])
        if cost_diff + cost_diff_ins > 0:
            continue
        if dur_diff_ins < idle_slot:
            if ((best_ins[1] < dur_diff_ins) or
                ((best_ins[1] == dur_diff_ins) and
                 (cost_diff + cost_diff_ins < best_ins[2]))):
                best_ins[0] = j
                best_ins[1] = dur_diff_ins
                best_ins[2] = cost_diff + cost_diff_ins
    return best_ins[0]


def one_move_reduce_idle(mdvrp, p1, quality, duration, setup_time, setup_cost,
                         demand):
    p_res = p1.clone()
    # all idle tasks
    idle = p_res.idle_slots.keys()
    constrained_tasks = set(p_res.all_constraints.nodes())
    for item in idle:
        if item not in p_res.idle_slots.keys():
            continue
        node = int(item[1:])
        [d, v, r, limit_up] = aux.find_node_route(p_res, node)
        if limit_up == len(p_res.routes[v][r]) - 1:
            continue
        if limit_up == 0:
            limit_low = 0
        else:
            con = [x for x in p_res.routes[v][r] if x in constrained_tasks]
            n_i = con.index(node)
            if n_i == 0:
                limit_low = 0
            else:
                limit_low = p_res.routes[v][r].index(con[n_i-1])

        for i in range(limit_up + 1, len(p_res.routes[v][r])):
            candidate = p_res.routes[v][r][i]
            if candidate in constrained_tasks:
                continue
            idle_slot = p_res.idle_slots[item][1] - p_res.idle_slots[item][0]
            best_ind = calc_insertion_reduce_idle(p_res, v, r, candidate, i,
                                                  limit_low, limit_up,
                                                  idle_slot, duration,
                                                  setup_time, setup_cost,
                                                  demand)
            if best_ind == -1:
                continue
            if not p_res.remove_node(v, r, i, quality[v], duration, setup_time,
                              demand[v], setup_cost[v]):
                return p1
            if not p_res.add_node(candidate, v, r, best_ind, quality[v], duration,
                                  setup_time, demand[v], setup_cost[v]):
                return p1
            p_res.evaluate_schedule(duration, setup_time)
            limit_up += 1
            if item not in p_res.idle_slots.keys():
                break
    return p_res


def two_swap(p1, depot_vehicles, borderline_customers, candidate_depots,
             quality, duration, setup_time, demand, setup_cost):

    borderline_customers_shuffle = deepcopy(borderline_customers)
    random.shuffle(borderline_customers_shuffle)
    for node1 in borderline_customers_shuffle:
        if node1 in p1.unserved_customers:
            continue

        [d1, v1, r1, n1_index] = aux.find_node_route(
            p1, node1, len(depot_vehicles[0]))

        [candidates, _] = get_swap_candidates(p1, d1, v1, node1,
                                              borderline_customers_shuffle,
                                              candidate_depots, depot_vehicles,
                                              demand)
        bestImprovement = None
        bestSwap = None
        for node2 in candidates:
            [d2, v2, r2, n2_index] = aux.find_node_route(
                p1, node2, len(depot_vehicles[0]))
            [success, p_tmp, cost] = swap_nodes(p1.clone(), v1, r1,
                                                n1_index, v2, r2,
                                                n2_index, quality, duration,
                                                setup_time, demand,
                                                setup_cost)
            if success:
                time_criteria = [prop.problem_criteria.TIME,
                                 prop.problem_criteria.MAKESPAN,
                                 prop.problem_criteria.MAKESPANCOST]
                cost_criteria = [prop.problem_criteria.COST,
                                 prop.problem_criteria.COSTMAKESPAN]
                if p1.criteria in cost_criteria:
                    imp = cost
                elif p1.criteria in time_criteria:
                    old_dur = p1.total_duration
                    p_tmp.evaluate_schedule(duration, setup_time)
                    imp = p_tmp.total_duration - old_dur
                if imp < 0:
                    if bestImprovement is None:
                        bestImprovement = imp
                        bestSwap = p_tmp.clone()
                    else:
                        if imp < bestImprovement:
                            bestImprovement = imp
                            bestSwap = p_tmp.clone()
        # swap the solution with best found
        if bestImprovement is not None:
            p1 = bestSwap.clone()
    return p1
