#!/usr/bin/env python

"""
Example of a specific node implementation of CBM_pop agent.
The evaluated examples are from Crodeau dataset.
"""

__author__ = 'barbanas'

from cbm_pop_lib.modules.cbm_pop_agent import CBMPopAgent
from cbm_pop_alg_mdvrp import CBMPopAlgorithmMDVRP
from cbm_pop_lib.cordeau_parser import cordeau_parser as parser
import rospy
import os
import rospkg

rospack = rospkg.RosPack()


class CBMPopAgentNode(CBMPopAgent):

    def __init__(self):
        CBMPopAgent.__init__(self, CBMPopAlgorithmMDVRP)

        # get data location
        pack = rospy.get_param('~data_pack')
        data_dir = rospy.get_param('~data_dir')
        self.data_dir = os.path.join(rospack.get_path(pack), "data/", data_dir)

        rospy.spin()

    def load_problem_structure(self, problem_id):
        # prepare self.mdvrp
        self.logger.info("Loading mission info..")
        filepath = os.path.join(self.data_dir, problem_id)
        self.mdvrp = parser.load_specification_cordeau(filepath)

    def process_final_solution(self):
        self.logger.info("Optimization runtime {}".format(self.alg_runtime))

        s = self.algorithm.best_sol_coalition.get_linearized_schedules(
            self.mdvrp.customer_labels)

        if self.algorithm.best_sol_coalition.check_prec():
            self.logger.info("Precedence OK.")
        else:
            self.logger.error("Broken precedence!")

        # uncomment to plot
        # self.algorithm.best_sol_coalition.plot(self.mdvrp)

        self.logger.info("Best solution ranking parameters: \n {}".format(
            self.algorithm.best_sol_coalition.get_ranking_params()))


if __name__ == "__main__":

    rospy.init_node("cbm_pop_agent")

    try:
        cbm_agent = CBMPopAgentNode()
    except rospy.ROSInterruptException:
        pass
