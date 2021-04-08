#!/usr/bin/env python

import sys
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from copy import deepcopy
import os
import rospkg
import random
import rospy

from cbm_pop_lib.operators.auxiliary_functions import find_node_route
from cbm_pop_lib.aux import my_logger
from cbm_pop_lib.common import cbm_alg_properties as prop

# Logger setup
logger = my_logger.CustomLogger()
logger.name = "Chromosome"


class Chromosome(object):
    """ TODO finish this..
    Description of Chromosome

    Attributes:
        routes (type):
        total_quality (type):
        total_cost (type):
        total_duration (type):
        max_vehicle_load (type):
        capacity (type):
        capacity[i] (type):
        max_vehicle_load[i] (type):
        idle_slots (type):
        start_times (type):
        end_times (type):
        fitness (type):
        unserved_customers (type):
        all_customers (type):
        prec_constraints (type):
        all_constraints (type):
        prec_matrix_indices (type):
        prec_matrix_nl (type):
        prec_matrix (type):
        prec_constraints, (type):
        prec_matrix (type):
        prec_matrix (type):
        prec_matrix_indices[x] (type):
        est_matrix (type):
        prec_matrix (type):
        prec_matrix_nl (type):
        est_matrix (type):

    Args:
        all_customers (list): list of all customers to be scheduled
        max_vehicle_load (np.array): maximal load of a vehicle
        precedence_constraints (nx.DiGraph): precedence constraint graph
        n (int): number of customers in a problem
        params (dictionary): algorithm parameters (problem variant and criteria)

    """

    def __init__(self, all_customers, max_vehicle_load, precedence_constraints,
                 time_windows, n, params=None):

        # key: vehicle label, value: list of routes (tasks are integer indices)
        self.routes = {}

        # solution properties
        self.total_quality = 0
        self.total_cost = 0
        self.total_duration = 0

        # initial max vehicle load
        self.max_vehicle_load = deepcopy(max_vehicle_load)

        # current capacity of each vehicle
        self.capacity = {}
        for i in range(len(self.max_vehicle_load)):
            self.capacity[i] = self.max_vehicle_load[i]

        # schedule properties
        self.idle_slots = {}
        self.start_times = np.zeros(n + 1)
        self.end_times = np.zeros(n + 1)

        # solution properties
        self.fitness = None
        self.unserved_customers = deepcopy(all_customers)
        self.all_customers = deepcopy(all_customers)

        # nx.digraph
        self.prec_constraints = deepcopy(precedence_constraints)
        # key: source_node, value: [constrained_node, offset_start, offset_end]
        self.sliding_time_windows = deepcopy(time_windows)
        # precedence + route constraints
        self.all_constraints = deepcopy(precedence_constraints)

        self.prec_matrix_indices = {}
        nl = list(self.prec_constraints.nodes())
        if len(nl) > 0:
            nl.sort()
            self.prec_matrix_nl = np.array(nl)  # nl = node list
            self.prec_matrix = nx.adjacency_matrix(self.prec_constraints, nl)
            self.prec_matrix.todense()
            self.prec_matrix = self.prec_matrix + np.eye(len(nl))
            i = 0
            for x in nl:
                self.prec_matrix_indices[x] = i
                i += 1
            self.est_matrix = np.zeros(len(nl))  # service earliest start time
            self.lft_matrix = np.zeros(len(nl))  # service latest finish time
        else:
            self.prec_matrix = None
            self.prec_matrix_nl = None
            self.est_matrix = None
            self.lft_matrix = None

        self.init_problem_params(params)

    def init_problem_params(self, params=None):
        """
        Initialize CBM problem parameters.

        Args:
            params=None (dictionary): algorithm parameters (problem variant and criteria)

        """
        if params is None:
            self.problem_variant = prop.problem_variants.CLASSIC

        else:
            variant = params["problem_variant"]
            if variant == 1:
                self.problem_variant = prop.problem_variants.OPEN
            else:
                self.problem_variant = prop.problem_variants.CLASSIC
            criteria = params["criteria"]
            if criteria == 2:
                self.criteria = prop.problem_criteria.COST
            elif criteria == 3:
                self.criteria = prop.problem_criteria.MAKESPAN
            elif criteria == 4:
                self.criteria = prop.problem_criteria.MAKESPANCOST
            else:
                self.criteria = prop.problem_criteria.TIME

    def plot(self, mdvrp, plot=True, save=False, figpath=""):
        """
        Plot the chromosome.

        Args:
            mdvrp (MDVRP): MDVRP problem instance
            plot=True (bool, optional): display plot figure
            save=False (bool, optional): save the figure
            figpath="" (str, optional): if save, location of file

        """
        plt.figure(figsize=(20, 20))
        mdvrp.draw()

        plt.title(str(self.get_ranking_params()))
        colors = cm.gist_rainbow(np.linspace(
            0, 1, num=len(self.routes.keys())))

        c = 0

        for vehicle in self.routes.keys():
            vehicle_label = mdvrp.vehicle_labels[vehicle]
            for route in self.routes[vehicle]:
                if len(route) > 0:
                    n = mdvrp.customer_labels[route[0] - 1]
                    line = plt.plot([mdvrp.vehicles[vehicle_label].position[0],
                                     mdvrp.nodes[n].position[0]],
                                    [mdvrp.vehicles[vehicle_label].position[1],
                                     mdvrp.nodes[n].position[1]],
                                    color=colors[c], linestyle='-')[0]
                    self.add_arrow(line)
                for i in range(1, len(route)):
                    n1 = mdvrp.customer_labels[route[i - 1] - 1]
                    n2 = mdvrp.customer_labels[route[i] - 1]

                    line = plt.plot([mdvrp.nodes[n1].position[0],
                                     mdvrp.nodes[n2].position[0]],
                                    [mdvrp.nodes[n1].position[1],
                                     mdvrp.nodes[n2].position[1]],
                                    color=colors[c], linestyle='-')[0]
                    self.add_arrow(line)
                if (len(route) > 0) and (self.problem_variant == prop.problem_variants.CLASSIC):
                    n = mdvrp.customer_labels[route[-1] - 1]
                    line = plt.plot([mdvrp.nodes[n].position[0],
                                     mdvrp.vehicles[vehicle_label].position[0]],
                                    [mdvrp.nodes[n].position[1],
                                     mdvrp.vehicles[vehicle_label].position[1]],
                                    color=colors[c], linestyle='-')[0]
                    self.add_arrow(line)
            c += 1
        if save:
            plt.savefig(figpath, format='eps', bbox_inches='tight', dpi=1000)
            plt.close()
        if plot:
            plt.show()
        else:
            plt.close()

    def add_arrow(self, line, position=None, size=15, color=None):
        """
        Add an arrow to a plotted line.

        Args:
            line (Line2D object): line object
            position=None (int): x-position of the arrow. If None, mean of xdata is taken
            size=15 (int): size of the arrow in fontsize points
            color=None (int): if None, line color is taken

        """
        if color is None:
            color = line.get_color()

        xdata = line.get_xdata()
        ydata = line.get_ydata()

        if position is None:
            position = xdata.mean()
        # find closest index
        start_ind = 0
        end_ind = start_ind + 1

        line.axes.annotate('',
                           xytext=(xdata[start_ind], ydata[start_ind]),
                           xy=(xdata[end_ind], ydata[end_ind]),
                           arrowprops=dict(arrowstyle="->", color=color),
                           size=size
                           )

    def get_linearized_schedules(self, node_labels=None):
        """
        Produce linearized schedules (in order of the schedule) of the chromosome solution.

        Args:
            node_labels=None (list): node labels to be used

        Returns:
            list: linearized schedule [[customer_label, start_time, end_time], ..]

        """
        sched = {}
        for vehicle in self.routes.keys():
            sched[vehicle] = []
            for route in self.routes[vehicle]:
                sched[vehicle].append([])
                for node in route:
                    if node_labels is not None:
                        label = node_labels[node - 1]
                    else:
                        label = str(node)
                    idle_key = "_" + str(node)
                    idle_label = "_" + label
                    if idle_key in self.idle_slots.keys():
                        sched[vehicle][-1].append([idle_label,
                                                   float(
                                                       self.idle_slots[idle_key][0]),
                                                   float(self.idle_slots[idle_key][1])])
                    sched[vehicle][-1].append([label, float(self.start_times[node]),
                                               float(self.end_times[node])])
        return sched

    def check_prec(self):
        """
        Check if precedence constraints are satisfied in the schedule.

        Returns:
            boolean: are prec constraints ok

        """
        if len(self.all_constraints.nodes()) == 0:
            return True
        for node in self.prec_matrix_indices.keys():
            n = self.prec_matrix_indices[node]
            if node not in self.unserved_customers:
                if self.est_matrix[n] - self.start_times[node] > 0.1:
                    return False
                if self.lft_matrix[n] > 0:
                    if self.end_times[node] - self.lft_matrix[n] > 0.1:
                        print [node, self.end_times[node], self.lft_matrix[n]]
                        return False
        return True

    def makespan(self):
        """
        Get makespan of the calculated plan (considering all routes
        of all vehicles).

        Returns:
            float: total solution makespan

        """
        return max(self.end_times) - min(self.start_times)

    def get_ranking_params(self):
        """
        Get parameters based on which the chromosome is evaluated.

        Returns:
            list: a list of ranking param values
        """
        if len(self.unserved_customers) > 0:
            return [sys.maxint]
        if self.criteria == prop.problem_criteria.TIME:
            return [self.total_duration]
        elif self.criteria == prop.problem_criteria.COST:
            return [self.total_cost]
        elif self.criteria == prop.problem_criteria.MAKESPAN:
            return [self.makespan()]
        elif self.criteria == prop.problem_criteria.MAKESPANCOST:
            return [self.makespan(), self.total_cost]

    def clone(self):
        """
        Clone the chromosome.

        Returns:
            Chromosome: a clone of chromosome

        """
        params = {}
        params["problem_variant"] = self.problem_variant
        params["criteria"] = self.criteria
        c = Chromosome(self.unserved_customers, self.max_vehicle_load,
                       self.prec_constraints, self.sliding_time_windows,
                       len(self.start_times) - 1, params)
        c.routes = deepcopy(self.routes)
        c.all_customers = deepcopy(self.all_customers)

        c.total_quality = self.total_quality
        c.total_cost = self.total_cost
        c.total_duration = self.total_duration

        c.capacity = deepcopy(self.capacity)

        c.idle_slots = deepcopy(self.idle_slots)
        c.start_times = deepcopy(self.start_times)
        c.end_times = deepcopy(self.end_times)

        c.all_constraints = deepcopy(self.all_constraints)
        c.prec_matrix = deepcopy(self.prec_matrix)

        return c

    def add_route(self, vehicle):
        """
        Add an empty route to the chromosome.

        Args:
            vehicle (int): vehicle id

        """
        if vehicle not in self.routes.keys():
            self.routes[vehicle] = []

        self.routes[vehicle].append([])

    def get_route_predecessors(self, route, index):
        """
        Gets predecessors of a node in based on a route ordering.

        Args:
            route (list): a route
            index (int): node index in a route

        Returns:
            list: node route predecessors

        """
        return list(set(route[:index]).intersection(self.prec_constraints.nodes()))

    def get_route_successors(self, route, index):
        """
        Gets successors of a node in based on a route ordering.

        Args:
            route (list): a route
            index (int): node index in a route

        Returns:
            list: node route successors

        """
        return list(set(route[index:]).intersection(self.prec_constraints.nodes()))

    def update_all_prec_insertion(self, route, node, insertion_index):
        """
        Update all precedence constraints based on the inserted node.

        Args:
            r (list): route
            node (int): node id
            insertion_index (int): insertion index

        """
        if len(self.all_constraints.nodes()) == 0:
            return

        route_pred = self.get_route_predecessors(route, insertion_index)
        np = set()
        for item in route_pred:
            if (item, node) not in self.all_constraints.edges():
                self.all_constraints.add_edge(item, node)
                np.add(item)

        for p in np:
            a = self.prec_matrix_indices[p]
            b = self.prec_matrix_indices[node]
            new_prec = self.prec_matrix[:, a] * self.prec_matrix[b, :]
            # new_prec[new_prec > 0] = 1
            self.prec_matrix += new_prec

        ns = set()
        if insertion_index < len(route) - 1:
            route_succ = self.get_route_successors(route, insertion_index + 1)
            for item in route_succ:
                if (node, item) not in self.all_constraints.edges():
                    self.all_constraints.add_edge(node, item)
                    ns.add(item)

        for s in ns:
            a = self.prec_matrix_indices[node]
            b = self.prec_matrix_indices[s]
            new_prec = self.prec_matrix[:, a] * self.prec_matrix[b, :]
            # new_prec[new_prec > 0] = 1
            self.prec_matrix += new_prec

    def update_all_prec_removal(self, route, removal_index):
        """
        Update all precedence constraints based on the removed node.

        Args:
            route (list): route
            removal_index (int): index of the removed node

        """
        node = route[removal_index]
        if node not in self.all_constraints.nodes():
            return

        route_pred = self.get_route_predecessors(route, removal_index)
        np = set()
        for item in route_pred:
            c = (item, node)
            # only detect route predecessors
            if (c not in self.prec_constraints.edges() and
                    c in self.all_constraints.edges()):
                self.all_constraints.remove_edge(item, node)
                np.add(item)

        for p in np:
            a = self.prec_matrix_indices[p]
            b = self.prec_matrix_indices[node]
            new_prec = self.prec_matrix[:, a] * self.prec_matrix[b, :]
            # new_prec[new_prec > 0] = 1
            self.prec_matrix -= new_prec

        route_succ = self.get_route_successors(route, removal_index + 1)
        ns = set()
        for item in route_succ:
            c = (node, item)
            if (c not in self.prec_constraints.edges() and
                    c in self.all_constraints.edges()):
                self.all_constraints.remove_edge(node, item)
                ns.add(item)

        for s in ns:
            a = self.prec_matrix_indices[node]
            b = self.prec_matrix_indices[s]
            new_prec = self.prec_matrix[:, a] * self.prec_matrix[b, :]
            # new_prec[new_prec > 0] = 1
            self.prec_matrix -= new_prec

    def add_node(self, node, vehicle, route, insertion_index, quality,
                 duration, setup_time, demand, setup_cost):
        """
        Add a node to the route.

        Args:
            node (int): node id
            vehicle (int): vehicle id
            route (int): route id
            insertion_index (int): insertion index within the route
            quality (np.matrix): quality matrix
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles
            demand (np.matrix): demand matrix
            setup_cost (np.matrix): setup cost matrix

        """
        if node in self.unserved_customers:
            self.unserved_customers.remove(node)
        else:
            logger.error("Can't add the node, already in chromosome!")
            logger.error("node {}".format(node))
            logger.error("unserved {}".format(self.unserved_customers))
            logger.error("routes {}".format(self.routes))
            sys.exit(0)  # TODO! return False

        prev_node = 0
        if insertion_index != 0:
            prev_node = self.routes[vehicle][route][insertion_index - 1]

        # calc cost
        cost_diff = setup_cost[prev_node][node] + demand[node]
        if insertion_index < len(self.routes[vehicle][route]):
            next_node = self.routes[vehicle][route][insertion_index]
        else:
            next_node = 0

        if (next_node > 0) or (self.problem_variant == prop.problem_variants.CLASSIC):
            cost_diff += (setup_cost[node][next_node]
                          - setup_cost[prev_node][next_node])

        # check capacity
        if self.capacity[vehicle] - cost_diff <= 0:
            return False

        self.routes[vehicle][route].insert(insertion_index, node)

        if node in self.prec_constraints.nodes():
            r = self.routes[vehicle][route]
            self.update_all_prec_insertion(r, node, insertion_index)

        # update quality
        self.total_quality += quality[node]

        # update cost
        self.total_cost += cost_diff
        self.capacity[vehicle] -= cost_diff

        return True

    def remove_nodes(self, nodes, quality, duration, setup_time, demand, setup_cost):
        """
        Remove nodes from the chromosome.

        Args:
            nodes (int list): a list of node ids
            quality (np.matrix): quality matrix for all vehicles
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles
            demand (np.matrix): demand matrix for all vehicles
            setup_cost (np.matrix): setup cost matrix for all vehicles

        """
        last_removed = -1
        for node in nodes:
            for vehicle in self.routes.keys():
                if last_removed == node:
                    break
                for r in range(len(self.routes[vehicle])):
                    if node in self.routes[vehicle][r]:
                        removal_index = self.routes[vehicle][r].index(node)
                        self.remove_node(
                            vehicle, r, removal_index, quality[vehicle],
                            duration, setup_time, demand[vehicle],
                            setup_cost[vehicle])
                        last_removed = node
                        break

    def remove_node(self, vehicle, route, removal_index, quality, duration,
                    setup_time, demand, setup_cost):
        """
        Remove the node from route.

        Args:
            vehicle (int): vehicle id
            route (int): route id
            removal_index (int): removal index within the route
            quality (np.matrix): quality matrix
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles
            demand (np.matrix): demand matrix
            setup_cost (np.matrix): setup cost matrix

        """
        node = self.routes[vehicle][route][removal_index]
        if removal_index == 0:
            prev_node = 0
        else:
            prev_node = self.routes[vehicle][route][removal_index - 1]

        if removal_index < len(self.routes[vehicle][route]) - 1:
            next_node = self.routes[vehicle][route][removal_index + 1]
        else:
            next_node = 0

        cost_diff = -(setup_cost[prev_node][node] + demand[node])
        if (next_node > 0) or (self.problem_variant == prop.problem_variants.CLASSIC):
            cost_diff += (setup_cost[prev_node][next_node] -
                          setup_cost[node][next_node])

        self.total_quality -= quality[node]
        self.total_cost += cost_diff
        self.capacity[vehicle] -= cost_diff

        self.unserved_customers.append(node)
        idle_label = "_" + str(node)
        if idle_label in self.idle_slots:
            del self.idle_slots[idle_label]
        self.start_times[node] = 0
        self.end_times[node] = 0

        r = self.routes[vehicle][route]
        self.update_all_prec_removal(r, removal_index)

        del self.routes[vehicle][route][removal_index]

    def update_successor_EST(self, node):
        """
        Update earliest start times for all successors of the node.

        Args:
            node (int): node id

        """
        if node not in self.all_constraints.nodes():
            return
        end_time = self.end_times[node]
        n = self.prec_matrix_indices[node]
        successors = np.where(self.prec_matrix[n, :])[1]
        mask = np.zeros(self.est_matrix.shape[0])
        mask[successors] = 1
        mask[n] = 0  # node shouldn't affect itself
        end_times = np.repeat(end_time, self.est_matrix.shape[0])
        end_times = np.multiply(end_times, mask)
        if node in self.sliding_time_windows:
            for item in self.sliding_time_windows[node]:
                i = self.prec_matrix_indices[item[0]]
                end_times[i] += item[1]
                # if there is latest finish time constraint
                if item[2] > 0:
                    if self.lft_matrix[i] == 0:
                        self.lft_matrix[i] = end_time + item[2]
                    else:
                        self.lft_matrix[i] = min(
                            self.lft_matrix[i], end_time + item[2])
        self.est_matrix = np.maximum(self.est_matrix, end_times)

    def evaluate_schedule(self, duration, setup_time):
        """
        Evaluate solution schedule.

        Args:
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles

        """
        self.initialize_schedule(duration, setup_time)
        self.adjust_schedule(duration, setup_time)

    def initialize_schedule(self, duration, setup_time):
        """
        Initialize solution schedule.

        Args:
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles

        """
        self.start_times *= 0
        self.end_times *= 0
        self.total_duration = 0
        for vehicle in self.routes.keys():
            for route in self.routes[vehicle]:
                prev_node = 0
                start_time = 0
                for node in route:
                    start_time += setup_time[vehicle][prev_node][node]
                    end_time = (start_time
                                + duration[vehicle][node])
                    self.start_times[node] = start_time
                    self.end_times[node] = end_time
                    prev_node = node
                    start_time = end_time
                if len(route) > 0:
                    self.total_duration += self.end_times[route[-1]]
                    if self.problem_variant == prop.problem_variants.CLASSIC:
                        self.total_duration += setup_time[vehicle][route[-1]][0]
        if self.est_matrix is not None:
            self.est_matrix *= 0
        if self.lft_matrix is not None:
            self.lft_matrix *= 0
        self.idle_slots = {}

    def adjust_schedule(self, duration, setup_time):
        """
        Adjust the schedule timing.

        Args:
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles

        """
        start = rospy.get_time()
        constraints = list(nx.topological_sort(self.all_constraints))
        for node in constraints:
            if node not in self.all_customers:
                continue
            n = self.prec_matrix_indices[node]
            if self.est_matrix[n] == 0:
                self.update_successor_EST(node)
                continue
            [_, v, r, index] = find_node_route(self, node)
            if v == -1:
                logger.error("Node not found in route!")
                logger.error("node {}".format(node))
                logger.error("unserved {}".format(self.unserved_customers))
                logger.error("routes {}".format(self.routes))
                sys.exit(0)
            self.adjust_route_schedule(v, r, index, duration[v], setup_time[v])
            self.update_successor_EST(node)

        dur = rospy.get_time() - start
        if dur > 0.1:
            logger.warn("Adjust schedule function duration {}".format(dur))

    def adjust_route_schedule(self, vehicle, start_route, start_index,
                              duration, setup_time):
        """
        Adjust route schedule for a vehicle.

        Args:
            vehicle (int): vehicle id
            start_route (int): start route index
            start_index (int): start index within the route
            duration (np.matrix): duration matrix of the vehicle
            setup_time (np.matrix): setup time matrix of the vehicle

        """
        s = rospy.get_time()
        node = self.routes[vehicle][start_route][start_index]
        n = self.prec_matrix_indices[node]

        if start_index == 0:
            if start_route == 0:
                prev_item = 0
            else:
                prev_item = self.routes[vehicle][start_route - 1][-1]
        else:
            prev_item = self.routes[vehicle][start_route][start_index - 1]

        sched_start = 0
        if prev_item != 0:
            sched_start += self.end_times[prev_item]

        change = 0
        if self.start_times[node] < self.est_matrix[n]:
            # shift start of the node because of prec constraint
            idle_time_key = "_" + str(node)
            self.idle_slots[idle_time_key] = [sched_start, self.est_matrix[n]]
            change = self.est_matrix[n] - self.start_times[node]

        if change == 0:
            return
        for route in range(start_route, len(self.routes[vehicle])):
            mask = np.zeros(self.end_times.shape[0], dtype=int)
            mask[self.routes[vehicle][route][start_index:]] = 1
            self.start_times += change * mask
            self.end_times += change * mask
        self.total_duration += change

        tm = rospy.get_time() - s
        if tm > 0.1:
            logger.warn(
                "Adjust route schedule function duration {}".format(tm))

    def insertion_minimal_cost(self, node, quality, duration, setup_time,
                               demand, setup_cost):
        """
        Insert node in the chromosome in the location with minimal cost.

        Args:
            node (int): node id
            quality (np.matrix): quality matrix
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles
            demand (np.matrix): demand matrix for all vehicles
            setup_cost (np.matrix): setup cost matrix for all vehicles

        """
        min_cost = []
        min_index = []
        makespan = []

        for vehicle in self.routes.keys():
            if self.capacity[vehicle] - demand[vehicle][node] <= 0:
                continue
            [min_cost_, min_index_, makespan_] = self.calc_min_insertion_cost(
                node, vehicle, duration[vehicle], setup_time[vehicle],
                demand[vehicle], setup_cost[vehicle])

            min_cost.append(min_cost_)
            min_index.append(min_index_)
            min_index[-1].insert(0, vehicle)
            makespan.append(makespan_)

        if len(min_cost) == 0:
            return False

        # this is for a single-criteria optimization
        minimum = min(min_cost)
        candidates = np.argwhere(np.array(min_cost) - minimum == 0)
        candidates = candidates.reshape(candidates.size)
        best_index = min_index[candidates[0]]

        best_vehicle = best_index[0]
        if minimum > 10e10:
            logger.warn(
                "Min cost > 10e10, not adding the node to the solution.")
            return False
        return self.add_node(node, best_vehicle, best_index[1], best_index[2],
                             quality[best_vehicle], duration, setup_time,
                             demand[best_vehicle], setup_cost[best_vehicle])

    def calc_possible_insertions(self, route, node):
        """
        Determine possible insertion locations for a node within a specific route.

        Args:
            route (list): route
            node (int): node id

        Returns:
            list: [a, b) - start and end index of possible insertions

        """
        if node not in self.all_constraints.nodes():
            return [0, len(route) + 1]

        nl = self.prec_matrix_nl
        n = self.prec_matrix_indices[node]
        pred = nl[np.where(self.prec_matrix[:, n])[0]]

        route_pred = [x for x in route if x in pred and x != node]
        a = 0
        if len(route_pred) > 0:
            a = route.index(route_pred[-1]) + 1

        succ = nl[np.where(self.prec_matrix[n, :])[1]]
        route_succ = [x for x in route if x in succ and x != node]
        b = len(route) + 1
        if len(route_succ) > 0:
            b = route.index(route_succ[0]) + 1

        return [a, b]

    def calc_insertion_cost(self, j, n, route, setup, static):
        """
        Calculate the cost of insertion of a node in

        Args:
            self (undefined):
            j (int): insertion index
            n (int): node id
            route (list): route
            setup (np.matrix): setup cost (time) matrix for the vehicle
            static (np.matrix): static cost (time) matrix for the vehicle

        Return:
            float: insertion cost

        """
        # calculate insertion cost
        if len(route) == 0:
            # if route is empty
            c = setup[0, n]
        else:
            if j == 0:
                c = (setup[0, n]
                     + setup[n, route[j]]
                     - setup[0, route[j]])
            elif j == len(route):
                c = setup[route[j - 1], n]
                if self.problem_variant == prop.problem_variants.CLASSIC:
                    c += (setup[n, 0] - setup[route[j - 1], 0])
            else:
                c = (setup[route[j - 1], n]
                     + setup[n, route[j]]
                     - setup[route[j - 1], route[j]])
        c += static[n]
        return c

    def calc_min_insertion_cost(self, n, vehicle, duration, setup_time,
                                demand, setup_cost):
        """
        Calculate minimal insertion cost location in the chromosome.

        Args:
            n (int): node id
            vehicle (int): vehicle id
            duration (np.matrix): duration matrix for all vehicles
            setup_time (np.matrix): setup time matrix for all vehicles
            demand (np.matrix): demand matrix for all vehicles
            setup_cost (np.matrix): setup cost matrix for all vehicles

        Returns:
            list: [minimal cost, index of min insertion, route makespan]

        """
        min_cost = None
        min_index = [-1, -1]
        time_criteria = [prop.problem_criteria.TIME,
                         prop.problem_criteria.MAKESPAN,
                         prop.problem_criteria.MAKESPANCOST]
        if (self.criteria in time_criteria):
            setup = setup_time
            static = duration
        elif self.criteria == prop.problem_criteria.COST:
            setup = setup_cost
            static = demand

        for i in range(len(self.routes[vehicle])):
            route = self.routes[vehicle][i]
            [a, b] = self.calc_possible_insertions(route, n)
            for j in range(a, b):
                # insertion cost at index j
                c = self.calc_insertion_cost(j, n, route, setup, static)
                if self.criteria != prop.problem_criteria.COST:
                    _cost = self.calc_insertion_cost(
                        j, n, route, setup_cost, demand)
                else:
                    _cost = c
                # check capacity
                if self.capacity[vehicle] - _cost <= 0:
                    continue

                if min_cost is None:
                    min_cost = c
                    min_index = [i, j]
                elif c < min_cost:
                    min_cost = c
                    min_index = [i, j]

        if min_cost is None:
            return [sys.maxint, [0, 0], 0]
        makespan = 0
        route = self.routes[vehicle][min_index[0]]
        if len(route) > 0:
            for i in range(len(route)):
                makespan += duration[route[i]]
        return [abs(min_cost), min_index, makespan + duration[n]]
