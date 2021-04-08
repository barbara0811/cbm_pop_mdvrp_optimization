#!/usr/bin/env python


import matplotlib.pyplot as plt
import matplotlib.cm as cm
import networkx as nx
import numpy as np

import rospkg
rospack = rospkg.RosPack()


class MDVRP(object):
    """ TODO finish this
    Description of MDVRP

    Attributes:
        n (type):
        m (type):
        k (type):
        depot_vehicles (type):
        depot_labels (type):
        vehicle_labels (type):
        customer_labels (type):
        nodes (type):
        vehicles (type):

    Args:
        k (undefined):
        n (undefined):
        m (undefined):

    """

    def __init__(self, k, n, m):

        self.n = n  # customer number
        self.m = m  # depot number
        self.k = k * m  # vehicle number

        self.depot_vehicles = {}
        self.depot_labels = []
        self.vehicle_labels = []
        self.customer_labels = []

        self.nodes = {}
        self.vehicles = {}

        # quality
        # k_ir, default quality is 1
        self.quality_matrix = np.ones((self.k, self.n + 1))
        # duration
        self.duration_matrix = np.zeros((self.k, self.n + 1))  # s_ir
        self.setup_duration_matrix = np.zeros(
            (self.k, self.n + 1, self.n + 1))  # t_ijr
        # cost
        self.demand_matrix = np.zeros((self.k, self.n + 1))  # q_ir
        self.setup_cost_matrix = np.zeros(
            (self.k, self.n + 1, self.n + 1))  # c_ijr

        # max vehicle load
        self.max_vehicle_load = np.zeros(self.k)

        self.precedence_graph = nx.DiGraph()
        # key: source_node, value: [constrained_node, offset_start, offset_end]
        self.sliding_time_windows = {}

        self.criteria = None
        self.problem_variant = None

    def load_precedence_constraints(self, filepath):
        """
        Load precedence constraints from a file.

        Args:
            filepath (string): path to file

        """
        self.precedence_graph = nx.DiGraph()

        with open(filepath, 'r') as f:
            for line in f.readlines():
                if len(line) == 0:
                    break
                labels = line[:-1].split(" ")
                self.add_precedence_constraint(labels[0], labels[1])
                if len(labels) == 4:
                    i = self.customer_labels.index(labels[0]) + 1
                    j = self.customer_labels.index(labels[1]) + 1
                    if i not in self.sliding_time_windows:
                        self.sliding_time_windows[i] = []
                    self.sliding_time_windows[i].append(
                        [j, float(labels[2]), float(labels[3])])

    def draw_clusters(self, customers, candidate_depots):
        """
        Draw grouping of customers and their candidate depots.

        Args:
            customers (list): customer id list
            candidate_depots (dictionary): candidate depots for each customer

        """
        plt.figure()
        self.draw()
        colors = cm.gist_rainbow(np.linspace(
            0, 1, num=len(self.depot_labels)))
        for c in customers:
            for depot in candidate_depots[c]:
                a = self.nodes[self.customer_labels[c - 1]]
                b = self.nodes[self.depot_labels[depot]]
                plt.plot([a.position[0], b.position[0]],
                         [a.position[1], b.position[1]],
                         color=colors[depot])
        plt.show()

    def input_precedece_constraints(self, constraints):
        """
        Input precedence constraints into the structure.

        Args:
            constraints (list): [[pred1, succ1], ..]

        """
        for item in constraints:
            self.add_precedence_constraint(item[0], item[1])

    def add_precedence_constraint(self, pred, succ):
        """
        Add a precedence constraint to the structure

        Args:
            pred (int): customer label
            succ (int): customer label

        """
        id1 = self.customer_labels.index(pred) + 1
        id2 = self.customer_labels.index(succ) + 1
        self.precedence_graph.add_node(id1)
        self.precedence_graph.add_node(id2)
        self.precedence_graph.add_edge(id1, id2)

        try:
            c = nx.find_cycle(self.precedence_graph)
            print c
            print "cycle!!"
        except:
            pass

    def draw(self):
        """
        Draw MDVRP problem.

        """
        plt.xlabel("x")
        plt.ylabel("y")
        for n in self.nodes.values():
            if n.type == "depot":
                plt.plot(n.position[0], n.position[1], "ro")
            else:
                plt.plot(n.position[0], n.position[1], "bo")

    def plot(self):
        """
        Plot MDVRP problem.

        """
        plt.figure()
        self.draw()
        plt.show()


class Node(object):
    """
    Description of static Node class.

    Attributes:
        position (list): [x, y, z] position of node
        type (string): type = ("customer", "depot")

    Args:
        x (float): node x position
        y (float): node y position
        t (string): node type
        z=0 (float): node z position

    """

    def __init__(self, x, y, t, z=0):
        self.position = [x, y, z]  # position [x, y, z]
        self.type = t  # type (depot, customer), for graphical representation


class Vehicle(object):
    """
    Description of Vehicle class.

    Attributes:
        position (list): [x, y, z] position of vehicle
        depot (string): depot label

    Args:
        x (float): vehicle x position
        y (float): vehicle y position
        z=0 (float): vehicle z position
        depot="" (string): depot label

    """

    def __init__(self, x, y, z=0, depot=""):
        self.position = [x, y, z]  # position [x, y, z]
        self.depot = depot
