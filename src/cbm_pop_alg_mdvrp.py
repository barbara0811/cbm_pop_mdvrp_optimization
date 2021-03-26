#!/usr/bin/env python
__author__ = 'barbanas'

from cbm_pop_lib.modules.cbm_pop_algorithm import CBMPopAlgorithm
from cbm_pop_lib.aux import my_logger
import cbm_pop_lib.operators.genetic_operators as op
import cbm_pop_lib.operators.auxiliary_functions as aux

import networkx as nx
import random


class CBMPopAlgorithmMDVRP(CBMPopAlgorithm):
    

    def __init__(self, params, ns_):
        CBMPopAlgorithm.__init__(self, params, ns_)

    def init_agent_states(self):
        """States:

        | s1 -- crossover operator has been applied
        | s2 -- diversification operator has been applied
        | s3 -- intensification operator two_opt
        | s4 -- intensification operator two_swap
        | s5 -- intensification operator one_move
        | s6 -- all intensification operators have been applied without modifying the solution 
        """
        # states depend on number of operators
        self.state_labels = ["s1", "s2", "s3", "s4", "s5", "s6"]

        self.operator_labels = ["crossover_bcrc", "crossover_bcrc_pop",
                                "mutation_intra_depot_reversal",
                                "mutation_intra_depot_swapping",
                                "mutation_inter_depot_swapping",
                                "mutation_single_customer_rerouting",
                                "two_opt", "two_swap", "one_move"]
        self.divers_num = 6
        self.intens_num = 3
        self.init_cycle_int = range(
            self.divers_num, self.divers_num + self.intens_num)

        self.op_score_co = [0] * (self.divers_num + self.intens_num)

    def action(self, mdvrp, o, p1):
        """Implementation of specific action calls for this problem
        (classic MDVRP with precedence constraints).

        Args:
            mdvrp (MDVRP): MDVRP problem instance
            o (int): operator to apply
            p1 (Chromosome): a solution to modify

        Returns:
            Chromosome: modified solution
        """
        self.op_score_co[o] += 1
        # apply the operator o
        if o < 2:
            if o == 0: 
                # best cost route crossover with best coalition solution
                p2 = self.best_sol_coalition
            elif o == 1:
                # choose a second parent
                p2_ind = aux.roulette_wheel(self.fitness)
                p2 = self.population[p2_ind]

            vehicle_candidates = []
            for v in p1.routes.keys():
                if len(p1.routes[v][0]) == 0 or len(p2.routes[v][0]) == 0:
                    continue
                vehicle_candidates.append(v)
            if len(vehicle_candidates) == 0:
                return p1
            offsprings = op.crossover_bcrc(vehicle_candidates,
                                           p1, p2, mdvrp.quality_matrix,
                                           mdvrp.duration_matrix,
                                           mdvrp.setup_duration_matrix,
                                           mdvrp.demand_matrix,
                                           mdvrp.setup_cost_matrix)
            compare = []
            # evaluate offsprings
            for i in range(len(offsprings)):
                if len(offsprings[i].unserved_customers) > 0:
                    continue
                try:
                    nx.find_cycle(offsprings[i].all_constraints)
                    continue
                except nx.exception.NetworkXUnfeasible:
                    pass
                offsprings[i].evaluate_schedule(mdvrp.duration_matrix,
                                                mdvrp.setup_duration_matrix)
                compare.append(offsprings[i])
            if len(compare) == 0:
                return p1
            f = aux.pareto_ranking_procedure_eval(compare)
            # # return best between children
            best = aux.best_solution_index(compare, f)
            if len(best) == 1:
                best_ind = best[0]
            else:
                best_ind = random.choice(best)
            # if len(compare) == 2:
            #     self.population[p2_ind] = compare[1 - best_ind]
            return compare[best_ind]

        elif o == 2:  # mutation intra depot reversal
            # choose a vehicle for mutation
            vehicle = random.choice(p1.routes.keys())
            offspring = op.mutation_intra_depot_reversal(
                p1, vehicle,
                mdvrp.duration_matrix,
                mdvrp.setup_duration_matrix,
                mdvrp.setup_cost_matrix)

        elif o == 3:  # mutation intra depot swapping
            depot = random.choice(mdvrp.depot_vehicles.keys())
            offspring = op.mutation_intra_depot_swapping(
                p1, depot,
                mdvrp.depot_vehicles[depot],
                mdvrp.quality_matrix,
                mdvrp.duration_matrix,
                mdvrp.setup_duration_matrix,
                mdvrp.demand_matrix,
                mdvrp.setup_cost_matrix)

        elif o == 4:
            offspring = op.mutation_inter_depot_swapping(
                p1, mdvrp.depot_vehicles,
                self.borderlineCustomers,
                self.candidateDepots,
                mdvrp.quality_matrix,
                mdvrp.duration_matrix,
                mdvrp.setup_duration_matrix,
                mdvrp.demand_matrix,
                mdvrp.setup_cost_matrix)

        elif o == 5:
            vehicle = random.choice(p1.routes.keys())
            offspring = op.mutation_single_customer_rerouting(
                vehicle, p1, mdvrp.quality_matrix, mdvrp.duration_matrix,
                mdvrp.setup_duration_matrix, mdvrp.setup_cost_matrix,
                mdvrp.demand_matrix)

        elif o == 6:
            offspring = op.two_opt(p1, mdvrp.duration_matrix,
                                   mdvrp.setup_duration_matrix,
                                   mdvrp.setup_cost_matrix)

        elif o == 7:
            offspring = op.two_swap(p1, mdvrp.depot_vehicles,
                                    self.borderlineCustomers2,
                                    self.candidateDepots2,
                                    mdvrp.quality_matrix,
                                    mdvrp.duration_matrix,
                                    mdvrp.setup_duration_matrix,
                                    mdvrp.demand_matrix,
                                    mdvrp.setup_cost_matrix)

        elif o == 8:
            offspring = op.one_move(p1, mdvrp.quality_matrix,
                                    mdvrp.duration_matrix,
                                    mdvrp.setup_duration_matrix,
                                    mdvrp.demand_matrix,
                                    mdvrp.setup_cost_matrix)

        elif o == 7:
            offspring = op.one_move_reduce_idle(
                mdvrp, p1, mdvrp.quality_matrix, mdvrp.duration_matrix,
                mdvrp.setup_duration_matrix, mdvrp.setup_cost_matrix,
                mdvrp.demand_matrix)

        else:
            return p1

        if offspring.check_prec():
            offspring.evaluate_schedule(mdvrp.duration_matrix,
                                        mdvrp.setup_duration_matrix)
            return offspring
        else:
            self.logger.warn("Offspring precedence constraints not satisfied.")
            return p1
