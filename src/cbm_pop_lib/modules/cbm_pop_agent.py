#!/usr/bin/env python

"""
Basic CBM_pop agent implementation.
For a specific use, please implement the abstract methods.
"""

__author__ = 'barbanas'

from cbm_pop_mdvrp.msg import BestSolution, WeightMatrix, MissionInfo, FloatArray
from std_msgs.msg import String, Empty
from rospy.numpy_msg import numpy_msg
from cbm_pop_lib.modules.cbm_pop_algorithm import CBMPopAlgorithm
from cbm_pop_lib.common.mdvrp import MDVRP
from math import sqrt
from abc import abstractmethod
from cbm_pop_lib.aux import my_logger


import os
import sys
import rospy
import numpy as np
import rospkg

rospack = rospkg.RosPack()


class CBMPopAgent(object):

    def __init__(self, algorithm_class):
        self.init_properties(algorithm_class)
        self.init_topics()

    def init_properties(self, algorithm_class):
        # Logger setup
        self.logger = my_logger.CustomLogger()
        self.logger.name = rospy.get_name()

        # Agent name
        self.agent_names = rospy.get_param('~name').split(',')
        self.logger.info("Agent names are {}".format(self.agent_names))

        self.agent_classes = []

        # Get cbm parameters location (package and path)
        self.cbm_params_path = rospy.get_param('~alg_params_path')
        self.cbm_params_pack = rospy.get_param('~alg_params_pack')
        params = os.path.join(rospack.get_path(
            self.cbm_params_pack), self.cbm_params_path)

        # Instantiate the algorithm with the specified parameters
        self.algorithm = algorithm_class(params, self.agent_names)
        self.init_mission_structures()

    def init_topics(self):
        """ Initialization of CBM agent topics.
        """
        # Mission start message
        self.start_sub = rospy.Subscriber(
            "optimize", String, self.optimize_cb)

        # Mission info pub and sub
        self.info_pub = rospy.Publisher("/vrp_mission_info", MissionInfo,
                                        queue_size=30)
        self.info_sub = rospy.Subscriber("/vrp_mission_info", MissionInfo,
                                         self.mission_info_cb)
        self.setup_duration_pub = rospy.Publisher("/setup_duration",
                                                  numpy_msg(FloatArray),
                                                  queue_size=10)
        self.setup_duration_sub = rospy.Subscriber("/setup_duration",
                                                   numpy_msg(FloatArray),
                                                   self.setup_duration_cb)
        self.setup_cost_pub = rospy.Publisher("/setup_cost",
                                              numpy_msg(FloatArray),
                                              queue_size=10)
        self.setup_cost_sub = rospy.Subscriber("/setup_cost",
                                               numpy_msg(FloatArray),
                                               self.setup_cost_cb)
        self.duration_pub = rospy.Publisher("/duration",
                                            numpy_msg(FloatArray),
                                            queue_size=10)
        self.duration_sub = rospy.Subscriber("/duration",
                                             numpy_msg(FloatArray),
                                             self.duration_cb)
        self.demand_pub = rospy.Publisher("/demand",
                                          numpy_msg(FloatArray),
                                          queue_size=10)
        self.demand_sub = rospy.Subscriber("/demand",
                                           numpy_msg(FloatArray),
                                           self.demand_cb)
        self.quality_pub = rospy.Publisher("/quality",
                                           numpy_msg(FloatArray),
                                           queue_size=10)
        self.quality_sub = rospy.Subscriber("/quality",
                                            numpy_msg(FloatArray),
                                            self.quality_cb)

        # CBM messages
        # Algorithm signals to the agent it's finished optimizing
        # (after that -- waiting until all agents are done)
        self.alg_done_sub = rospy.Subscriber(
            "alg_done", String, self.finished_cb)
        # Signal that the planning procedure is done
        self.done_pub = rospy.Publisher("done_planning", Empty, queue_size=1)

        self.logger.info("initialized topics")

    def optimize_cb(self, msg):

        self.status_update(self.agent_names, "start")
        rospy.sleep(1)

        self.load_problem_structure(msg.data)
        if len(self.mdvrp_dict.keys()) > 0:
            self.share_mdvrp()

        self.status_update(self.agent_names, "ready")
        while self.status != "ready":
            rospy.sleep(0.2)
            print ". "

        self.merge_mdvrp()

        self.configure_problem_structure(msg.data)

        self.status_update(self.agent_names, "configured")
        while self.status != "configured":
            rospy.sleep(0.2)
            print ".."

        if self.mdvrp is not None:
            start = rospy.get_time()
            self.algorithm.optimize(self.mdvrp)
            self.alg_runtime = rospy.get_time() - start
            self.finish_mission()
        else:
            self.logger.warn("No mdvrp structure to optimize.")

    @abstractmethod
    def load_problem_structure(self, problem_id):
        # prepare mdvrp_dict
        self.logger.warn(("No implementation of problem structure definition." +
                          " For full functionality, implement specific cbm" +
                          " agent as in example scenario."))

    @abstractmethod
    def configure_problem_structure(self, problem_id):
        """Configure problem structure after all agents are known.
        This function is for problems where the mission is assessed online.
        """
        self.logger.warn(("No implementation of problem structure definition." +
                          " For full functionality, implement specific cbm" +
                          " agent as in example scenario."))

    def share_mdvrp(self):

        for name in self.agent_names:
            print self.agent_names
            mdvrp = self.mdvrp_dict[name]
            tmp = mdvrp.setup_duration_matrix[0]
            tmp = np.reshape(tmp, (tmp.size))
            self.setup_duration_pub.publish(tmp)

            tmp = mdvrp.setup_cost_matrix[0]
            tmp = np.reshape(tmp, (tmp.size))
            self.setup_cost_pub.publish(tmp)

            tmp = mdvrp.duration_matrix[0]
            np.reshape(tmp, (tmp.size))
            self.duration_pub.publish(tmp)

            tmp = mdvrp.demand_matrix[0]
            np.reshape(tmp, (tmp.size))
            self.demand_pub.publish(tmp)

            tmp = mdvrp.quality_matrix[0]
            np.reshape(tmp, (tmp.size))
            self.quality_pub.publish(tmp)

    def finish_mission(self):
        self.logger.info("Mission planning done.")
        self.process_final_solution()
        # Reinitialize mission structures
        self.algorithm.initialize_structures()
        self.init_mission_structures()
        self.done_pub.publish(Empty())

    @abstractmethod
    def process_final_solution(self):
        self.logger.warn(("No implementation of processing of final solution." +
                          " For full functionality, implement specific cbm" +
                          " agent as in example scenario."))

    def init_mission_structures(self):
        self.mdvrp = None
        self.mdvrp_dict = {}
        self.setupDurationOffset = {}
        self.setupCostOffset = {}
        self.durationOffset = {}
        self.demandOffset = {}
        self.qualityOffset = {}

        self.robot_positions = {}
        self.robot_status = []
        self.all_robot_classes = set()
        self.position = [-1, -1, -1]
        self.status = "no_mission"
        self.logger.info("Initialized mission structures")

    def status_update(self, names, status):
        for name in names:
            msg_out = MissionInfo()
            msg_out.agent = name
            msg_out.ag_class = self.agent_classes
            if self.position[2] > 0:
                msg_out.position = self.position
            msg_out.status = status
            self.info_pub.publish(msg_out)

    def mission_info_cb(self, msg):
        if msg.status == "start":
            self.robot_positions[msg.agent] = msg.position
            self.robot_status.append(0)
            self.all_robot_classes.update(set(msg.ag_class))
            self.status = "start"
        if msg.status == "ready":
            labels = self.robot_positions.keys()
            self.robot_status[labels.index(msg.agent)] = 1
            if min(self.robot_status) == 1:
                self.status = "ready"
        if msg.status == "configured":
            labels = self.robot_positions.keys()
            self.robot_status[labels.index(msg.agent)] = 2
            if min(self.robot_status) == 2:
                self.status = "configured"
        elif msg.status == "done":
            labels = self.robot_positions.keys()
            self.robot_status[labels.index(msg.agent)] = 3
            if min(self.robot_status) == 3:
                self.algorithm.done = True
                self.status = "done"

    def finished_cb(self, msg):
        self.status_update([msg.data], "done")

    def merge_mdvrp(self):
        agents = self.mdvrp_dict.keys()
        agents.sort()
        print agents
        print "..."
        all_tasks = self.mdvrp_dict[self.agent_names[0]].customer_labels
        self.mdvrp = MDVRP(1, len(all_tasks), len(agents))
        for i in range(len(agents)):
            ag = agents[i]
            self.mdvrp.setup_duration_matrix[i] = self.mdvrp_dict[ag].setup_duration_matrix[0]
            self.mdvrp.setup_cost_matrix[i] = self.mdvrp_dict[ag].setup_cost_matrix[0]
            self.mdvrp.duration_matrix[i] = self.mdvrp_dict[ag].duration_matrix[0]
            self.mdvrp.demand_matrix[i] = self.mdvrp_dict[ag].demand_matrix[0]
            self.mdvrp.quality_matrix[i] = self.mdvrp_dict[ag].quality_matrix[0]
            self.mdvrp.depot_vehicles[i] = [i]
            #TODO share this!
            self.mdvrp.max_vehicle_load[i] = sys.maxint #self.mdvrp_dict[ag].max_vehicle_load[0]

        self.mdvrp.customer_labels = all_tasks
        self.mdvrp.depot_labels = ['_' + r for r in self.robot_positions.keys()]
        self.mdvrp.vehicle_labels = [r for r in self.robot_positions.keys()]

        self.mdvrp.precedence_graph = self.mdvrp_dict[self.agent_names[0]].precedence_graph

    def setup_duration_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        n = int(sqrt(msg.data.size))
        if sender not in self.mdvrp_dict.keys():
            self.mdvrp_dict[sender] = MDVRP(1, n - 1, 1)
        self.mdvrp_dict[sender].setup_duration_matrix[0] = np.reshape(msg.data, (n, n))

    def setup_cost_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        n = int(sqrt(msg.data.size))
        if sender not in self.mdvrp_dict.keys():
            self.mdvrp_dict[sender] = MDVRP(1, n - 1, 1)
        self.mdvrp_dict[sender].setup_cost_matrix[0] = np.reshape(msg.data, (n, n))

    def duration_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        if sender not in self.mdvrp_dict.keys():
            self.mdvrp_dict[sender] = MDVRP(1, msg.data.size - 1, 1)
        self.mdvrp_dict[sender].duration_matrix[0] = msg.data

    def demand_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        if sender not in self.mdvrp_dict.keys():
            self.mdvrp_dict[sender] = MDVRP(1, msg.data.size - 1, 1)
        self.mdvrp_dict[sender].demand_matrix[0] = msg.data

    def quality_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        if sender not in self.mdvrp_dict.keys():
            self.mdvrp_dict[sender] = MDVRP(1, msg.data.size - 1, 1)
        self.mdvrp_dict[sender].quality_matrix[0] = msg.data
