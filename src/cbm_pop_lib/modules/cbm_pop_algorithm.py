#!/usr/bin/env python
__author__ = 'barbanas'

import cbm_pop_lib.operators.auxiliary_functions as aux
import cbm_pop_lib.operators.generational_functions as gen
from cbm_pop_lib.common.chromosome import Chromosome
from cbm_pop_lib.common import cbm_alg_properties as prop

import networkx as nx
import matplotlib.pyplot as plt
import json
import xml.etree.ElementTree

import random
from copy import deepcopy
from abc import abstractmethod
import numpy as np
import rospy
from cbm_pop_mdvrp.msg import BestSolution, WeightMatrix
from std_msgs.msg import Empty, String
from cbm_pop_lib.aux import my_logger


class CBMPopAlgorithm:

    def __init__(self, params, ns_):

        self.logger = my_logger.CustomLogger()
        self.logger.name = "CBM_alg"

        self.init_agent_states()
        self.load_parameters(params)
        self.initialize_structures()

        # Best solution and weight matrix
        self.best_solution_pub = rospy.Publisher(
            "/best_solution", BestSolution, queue_size=20)
        self.weight_matrix_pub = rospy.Publisher(
            "/weight_matrix", WeightMatrix, queue_size=20)

        self.best_solution_sub = rospy.Subscriber(
            "/best_solution", BestSolution, self.best_solution_cb)
        self.weight_matrix_sub = rospy.Subscriber(
            "/weight_matrix", WeightMatrix, self.weight_matrix_cb)

        self.done_pub = rospy.Publisher("alg_done", String, queue_size=5)
        self.robot_names = ns_

    @abstractmethod
    def init_agent_states(self):
        self.logger.error("Specific agent states not implemented..")
        raise NotImplementedError

    def initialize_structures(self, pop_size=None):

        if pop_size is None:
            pop_size = self.pop_size

        self.wait_best_sol_buffer = False
        self.best_sol_buffer = []
        # experience memory
        # stores the values of state, operator and gain for
        # each operator in D-I cycle
        self.H = []  # [[state, operator, gain], [], ...]

        # weight matrix (state-operator)
        self.weight_matrix = np.zeros(
            (len(self.state_labels), self.divers_num + self.intens_num))
        self.weight_matrix[:] = 1  # initial value
        # after crossover - mutation or crossover
        self.weight_matrix[0][self.divers_num:] = 0
        self.weight_matrix[1][:self.divers_num] = 0  # after D has to come I
        for i in range(self.intens_num):
            # after once in I, only I can be applied
            self.weight_matrix[i + 2][:self.divers_num] = 0
            # same op can't be applied twice
            self.weight_matrix[i + 2][self.divers_num + i] = 0
        # at the end of D-I cycle, a crossover operator has to be applied
        self.weight_matrix[-1][2:] = 0

        # solutions
        self.population = []
        self.best_solution = None
        self.best_sol_coalition = None

        self.fitness = None

        # experience exchange
        self.best_solution_sub = None
        self.weight_matrix_sub = None
        self.best_sol_buffer = []
        self.wait_best_sol_buffer = False

        self.done = False

    def load_parameters(self, filepath):

        tree = xml.etree.ElementTree.parse(filepath)
        root = tree.getroot()
        p = root.find("parameters")

        self.alpha = float(p.find("alpha").text)
        self.sigma = [float(x) for x in p.find(
            "sigma").text.split(' ') if len(x) > 0]
        self.rho = float(p.find("rho").text)

        self.pop_size = int(p.find("pop_size").text)
        self.iter_num = int(p.find("iter_num").text)
        self.cycle_no_change = int(p.find("cycle_no_change").text)
        self.timeout = int(p.find("timeout_s").text)

        variant = int(p.find("problem_variant").text)
        criteria = int(p.find("criteria").text)
        self.problem_params = {}
        self.problem_params["problem_variant"] = variant
        self.problem_params["criteria"] = criteria

    def optimize(self, mdvrp):

        self.init_population(mdvrp)
        self.evaluate_population(mdvrp)
        self.logger.info("Population initialized and evaluated.")
        best = aux.best_solution_index(self.population, self.fitness)
        if len(best) == 1:
            best_ind = best[0]
        else:
            best_ind = random.choice(best)
        self.best_solution = self.population[best_ind].clone()
        self.best_sol_buffer.append(self.best_solution)

        time_criteria = [prop.problem_criteria.TIME,
                         prop.problem_criteria.MAKESPAN,
                         prop.problem_criteria.MAKESPANCOST]
        if self.population[0].criteria in time_criteria:
            setup = mdvrp.setup_duration_matrix
        elif self.population[0].criteria == prop.problem_criteria.COST:
            setup = mdvrp.setup_cost_matrix
        [self.borderlineCustomers, self.candidateDepots] = aux.calc_borderline_customers(
            mdvrp.k / mdvrp.m, setup, 0.6)
        [self.borderlineCustomers2, self.candidateDepots2] = aux.calc_borderline_customers(
            mdvrp.k / mdvrp.m, setup, 0.4)

        # put to True to draw clusters
        if False:
            mdvrp.draw_clusters(self.borderlineCustomers, self.candidateDepots)

        iteration = 0
        # intensification operators to apply in one cycle
        # without modifying a solution to reach local optimum
        self.cycle_int = deepcopy(self.init_cycle_int)

        # choose a starting solution to explore
        p1_ind = aux.roulette_wheel(self.fitness)
        current_sol = self.population[p1_ind]

        ms = []
        bst_ms = []

        t_s = rospy.get_time()
        done_calc = 0
        cycle_num = 0
        cycle_last_change = 0

        while True:
            '''
            Perform one iteration of CBM agent.
            '''
            if iteration == self.iter_num:
                self.logger.info("Done, max iteration reached.")
                done_calc = 1
            if done_calc == 0:
                if cycle_num - cycle_last_change > self.cycle_no_change:
                    self.logger.info("Done, no improvement.")
                    done_calc = 1
                if rospy.get_time() - t_s > self.timeout:
                    self.logger.info("Done, timeout.")
                    done_calc = 1

            self.process_best_solutions(mdvrp)
            # calculate current state
            s = self.perception()

            # at the beginning of a cycle choose a solution to modify
            if s == len(self.state_labels) - 1:
                cycle_num += 1
                # process best coalition solutions received from other agents
                prev_best_sol_coalition = self.best_sol_coalition.clone()

                if (cycle_num - cycle_last_change) % 10 == 0:
                    self.population[p1_ind] = current_sol.clone()
                    self.evaluate_population(mdvrp)
                    p1_ind = aux.roulette_wheel(self.fitness)
                    current_sol = self.population[p1_ind].clone()

            # choose an action to perform
            o = self.deliberation(self.cycle_int, s)

            # perform action on the chosen soution
            new_solution = self.action(mdvrp, o, current_sol)

            # evaluate gain
            compare = [current_sol, new_solution]
            f = aux.pareto_ranking_procedure_eval(compare)
            gain = f[1] - f[0]
            if o in self.init_cycle_int:  # if intensification was applied
                if gain <= 0.01:
                    self.cycle_int.remove(o)
                else:
                    # gain resets the intensification cycle
                    self.cycle_int = deepcopy(self.init_cycle_int)

            # Update history
            self.H.append([s, o, gain])

            # Update solutions
            current_sol = new_solution.clone()
            # ms.append(current_sol.get_ranking_params()[1])
            # bst_ms.append(self.best_sol_coalition.get_ranking_params()[1])

            # check for the end of D-I cycle
            # if there is no intensification operator to be applied
            if len(self.cycle_int) == 0:
                if cycle_num == 5:
                    self.broadcast_solution(self.best_solution)
                # if best found solution improved by agent in D-I cycle
                # self.population[p1_ind] = new_solution.clone()
                comp = [new_solution, self.best_solution,
                        prev_best_sol_coalition]
                f = aux.pareto_ranking_procedure_eval(comp)

                best = aux.best_solution_index(comp, f)
                found_best_coal = False
                if len(best) == 1:
                    if best[0] == 0:
                        found_best_coal = True
                        self.logger.info("Improved coalition best, ranking params: {}".format(
                            new_solution.get_ranking_params()))
                        # improved coalition and own best
                        self.best_solution = new_solution.clone()
                        self.individual_learning(1)
                        matrix = self.weight_matrix.tolist()
                        self.broadcast_weight_matrix(matrix)
                        self.best_sol_buffer.append(new_solution)
                        self.broadcast_solution(new_solution)
                        cycle_last_change = cycle_num

                best = aux.best_solution_index(comp[:2], f[:2])
                if len(best) == 1 and not found_best_coal:
                    if best[0] == 0:
                        self.logger.info("Improved own best solution.")
                        # improved own best solution
                        self.best_solution = new_solution.clone()
                        self.individual_learning(0)
                        cycle_last_change = cycle_num
                # reset cycle structures
                self.cycle_int = deepcopy(self.init_cycle_int)
                self.H = []

            if iteration % 100 == 0:
                self.logger.info([iteration, cycle_num, cycle_last_change,
                                  self.best_solution.get_ranking_params()[0],
                                  self.best_sol_coalition.get_ranking_params()[0]])

            iteration += 1

            if done_calc == 1:
                for name in self.robot_names:
                    msg = String()
                    msg.data = name
                    self.done_pub.publish(msg)
                    rospy.sleep(0.1)
                done_calc = 2
            if self.done == 1:
                break

        self.broadcast_solution(self.best_sol_coalition)

        rospy.sleep(0.1)
        self.logger.info("Weight matrix:\n{}".format(self.weight_matrix))

        self.process_best_solutions(mdvrp)
        self.done = 2
        if self.best_sol_coalition is None:
            self.best_sol_coalition = self.best_solution

        self.logger.info("Operator score: {}".format(self.op_score_co))

    ############################
    # Initialization function  #
    ############################

    def init_population(self, mdvrp):
        """
        A method that creates initial population of feasible solutions to
        the scheduling problem.

        Args:
            mdvrp - a problem to solve
        """
        self.population = []
        self.fitness = []
        print self.problem_params
        no_cycle = 0
        while len(self.population) < self.pop_size:
            self.population.append(
                gen.greedy_insertion(mdvrp, self.problem_params))
            try:
                c = nx.find_cycle(self.population[-1].all_constraints)
                self.logger.warn(
                    "Cycle in precedence graph! Rejecting the solution.")
                self.logger.warn("Cycle: {}".format(c))
                del self.population[-1]
            except nx.exception.NetworkXUnfeasible:
                no_cycle += 1

    def evaluate_population(self, mdvrp):
        for sol in self.population:
            sol.evaluate_schedule(mdvrp.duration_matrix,
                                  mdvrp.setup_duration_matrix)
        self.fitness = aux.pareto_ranking_procedure_eval(self.population)

    ###################
    # Perception      #
    ###################

    def perception(self):
        # compute the current state s
        if len(self.H) == 0:  # if experience memory is empty
            s = len(self.state_labels) - 1  # the last state was local optimum
        else:
            prev_op = self.H[-1][1]
            prev_state = self.H[-1][0]

            if prev_op < self.divers_num:  # if diversification was applied
                if prev_state == 0:
                    s = 1
                else:
                    s = 0
            else:  # if intensification was applied
                s = prev_op - self.divers_num + 2
        return s

    ###################
    # Deliberation    #
    ###################

    def calc_probability(self, values):

        prob = deepcopy(values)
        prob /= np.sum(prob)
        return prob

    def deliberation(self, cycle_int, s):
        # decide of an operator o to apply considering s
        prob = self.calc_probability(self.weight_matrix[s])
        not_ok = True
        while not_ok:
            o = aux.roulette_wheel(prob)
            if o not in self.init_cycle_int:  # not intensifier -- OK
                not_ok = False
            elif o in cycle_int:  # intensifier --> check if it is allowed
                not_ok = False
        return o

    ###################
    # Action          #
    ###################

    @ abstractmethod
    def action(self, mdvrp, o, p1):
        self.logger.error("Specific actions not implemented..")
        raise NotImplementedError

    ############################
    # Learning mechanisms      #
    ############################

    def individual_learning(self, i):
        for item in self.H:
            self.weight_matrix[item[0]][item[1]] += self.sigma[i]

    def mimetism_learning(self, matrix):
        self.weight_matrix = np.add(np.multiply(
            self.weight_matrix, (1 - self.rho)), np.multiply(matrix, self.rho))

    ############################
    # Solution exchange        #
    ############################

    def broadcast_solution(self, solution):

        msg = BestSolution()
        msg.agent = rospy.get_namespace()
        msg.json_routes = json.dumps(solution.routes)
        msg.total_quality = solution.total_quality
        msg.total_cost = solution.total_cost
        msg.json_capacity = json.dumps(solution.capacity)
        msg.unserved_customers = solution.unserved_customers
        msg.json_all_prec = json.dumps(list(solution.all_constraints.edges()))
        if solution.prec_matrix is not None:
            n = solution.prec_matrix.size
            prec = json.dumps(np.reshape(solution.prec_matrix, n).tolist())
            msg.json_prec_matrix = prec
        self.best_solution_pub.publish(msg)

    def broadcast_weight_matrix(self, matrix):

        msg = WeightMatrix()
        msg.agent = rospy.get_namespace()
        msg.json_matrix = json.dumps(matrix)

        self.weight_matrix_pub.publish(msg)

    def best_solution_cb(self, msg):

        if msg.agent == rospy.get_namespace():
            return
        if len(self.population) == 0:
            self.logger.warn("Population not yet initialized.")
            return

        params = {}
        params["problem_variant"] = self.population[0].problem_variant
        params["criteria"] = self.population[0].criteria
        # basic precedence constraints are copied from this agent's solutions..
        solution = Chromosome(list(msg.unserved_customers),
                              self.population[0].max_vehicle_load,
                              self.population[0].prec_constraints,
                              self.population[0].sliding_time_windows,
                              len(self.population[0].start_times) - 1, params)
        solution.all_customers = self.population[0].all_customers
        routes = json.loads(msg.json_routes)
        capacity = json.loads(msg.json_capacity)
        all_prec = json.loads(msg.json_all_prec)
        for item in all_prec:
            solution.all_constraints.add_edge(item[0], item[1])

        for d in routes.keys():
            solution.routes[int(d)] = routes[d]
            solution.capacity[int(d)] = capacity[d]

        if solution.prec_matrix is not None:
            s = solution.prec_matrix.shape
            prec = np.matrix(json.loads(msg.json_prec_matrix)).reshape(s)
            solution.prec_matrix = prec

        solution.total_quality = msg.total_quality
        solution.total_cost = msg.total_cost

        while self.wait_best_sol_buffer:
            rospy.sleep(0.1)
        self.best_sol_buffer.append(solution)

    def process_best_solutions(self, mdvrp):

        if len(self.best_sol_buffer) == 0:
            return
        self.wait_best_sol_buffer = True
        compare = deepcopy(self.best_sol_buffer)
        self.best_sol_buffer = []
        self.wait_best_sol_buffer = False

        for sol in compare:
            sol.evaluate_schedule(mdvrp.duration_matrix,
                                  mdvrp.setup_duration_matrix)
        if self.best_sol_coalition is not None:
            compare.append(self.best_sol_coalition)

        f = aux.pareto_ranking_procedure_eval(compare)
        best = aux.best_solution_index(compare, f)
        self.best_sol_coalition = compare[random.choice(best)]

    def weight_matrix_cb(self, msg):

        if msg.agent == rospy.get_namespace():
            return

        wm = json.loads(msg.json_matrix)
        wm = np.array(wm)

        # mimetism learning
        self.mimetism_learning(wm)
