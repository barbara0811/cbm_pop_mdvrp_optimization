#!/usr/bin/env python

from copy import deepcopy
from math import sqrt
import random
import numpy as np


def find_node_route(chromosome, node1, vehicles_per_depot=None):
    v1 = -1
    d1 = -1
    r1 = -1
    n1_index = -1
    # find a route where this node is
    for v in chromosome.routes.keys():
        for r in range(len(chromosome.routes[v])):
            if node1 in chromosome.routes[v][r]:
                v1 = v
                if vehicles_per_depot is not None:
                    d1 = v / vehicles_per_depot
                r1 = r
                n1_index = chromosome.routes[v][r1].index(node1)
                break
    return [d1, v1, r1, n1_index]


def calc_borderline_customers(vehicles_per_depot, setup, tolerance):
    borderline_customers = []
    candidate_depots = {}

    depot_dist = np.zeros(
        (setup.shape[1] - 1, setup.shape[0] / vehicles_per_depot))

    for depot in range(setup.shape[0] / vehicles_per_depot):
        first_vehicle = depot * vehicles_per_depot
        tmp = setup[first_vehicle, 0, :]
        tmp = np.delete(tmp, 0)
        depot_dist[:, depot] = tmp

    clustering_criteria = np.zeros(depot_dist.shape)
    for customer in range(setup.shape[1] - 1):
        min_dist = min(depot_dist[customer, :])
        if min_dist > 0:
            clustering_criteria[customer, :] = (
                depot_dist[customer, :] - min_dist) / min_dist
        depot_candidates = np.where(clustering_criteria[customer, :] < tolerance)
        if len(depot_candidates[0]) > 1:
            borderline_customers.append(customer + 1)
            candidate_depots[customer + 1] = [d for d in depot_candidates[0]]

    return [borderline_customers, candidate_depots]


def pareto_ranking_procedure_eval(population):

    fitness = []
    rankingParams = [np.array(x.get_ranking_params()) for x in population]

    dominanceMatrix = np.zeros((len(population), len(population)))
    dist = np.zeros((len(population), len(population)))

    param_num = len(rankingParams[-1])
    for i in range(len(population)):
        for j in range(len(population)):
            if j == i:
                continue
            if j > i:
                dist[i][j] = 0
                for k in range(param_num):
                    dist[i][j] += pow(rankingParams[i]
                                      [k] - rankingParams[j][k], 2)
                dist[i][j] = sqrt(dist[i][j])
                dist[j][i] = dist[i][j]

            nonDominated = True
            for k in range(param_num):
                if rankingParams[j][k] < rankingParams[i][k]:
                    if abs(rankingParams[j][k] - rankingParams[i][k]) > 0.05:
                        nonDominated = False
                        break

            # if solution j dominates i
            if not nonDominated:
                dominanceMatrix[j][i] = 1

    for j in range(len(population)):
        rawFitness = sum(dominanceMatrix[:, j])
        a = deepcopy(dist[j])
        np.delete(a, j)
        density = 1.0 / (np.min(a) + 2)
        population[j].fitness = 1.0 / (rawFitness + density + 1)
        fitness.append(population[j].fitness)

    return fitness


def best_solution_index(compare, fitness):

    m = max(fitness)
    maxFitInd = [i for i in range(len(fitness)) if abs(fitness[i] - m) < 0.05]
    if len(maxFitInd) == 1:
        return [maxFitInd[0]]
    best1 = maxFitInd

    # first (MIN)
    primary_ranking_param = [compare[i].get_ranking_params()[0] for i in best1]
    m = min(primary_ranking_param)
    min_ind1 = [best1[i] for i in range(
        len(primary_ranking_param)) if abs(primary_ranking_param[i] - m) < 0.1]
    best2 = list(set(best1) & set(min_ind1))

    if len(best2) == 0:
        return best1
    elif len(best2) == 1:
        return [best2[0]]

    if len(compare[i].get_ranking_params()) == 1:
        return best1

    # second (MIN)
    secondary_ranking_param = [compare[i].get_ranking_params()[1]
                               for i in best2]
    m = min(secondary_ranking_param)
    min_ind2 = [best2[i] for i in range(
        len(secondary_ranking_param)) if abs(secondary_ranking_param[i] - m) < 0.1]
    best3 = list(set(best2) & set(min_ind2))

    if len(best3) == 0:
        return best2
    elif len(best3) == 1:
        return [best3[0]]
    else:
        return best3


def roulette_wheel(fitness):

    s = sum(fitness)
    prob_sum = 0

    win = random.random()
    for i in range(len(fitness)):
        prob_sum += fitness[i] / s
        if win <= prob_sum:
            return i
