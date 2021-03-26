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
from math import sqrt
from abc import abstractmethod
from cbm_pop_lib.aux import my_logger

import os
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
        # Signal to the caller that the planning procedure is done
        self.done_pub = rospy.Publisher("done", Empty, queue_size=1)

        self.logger.info("initialized topics")

    def optimize_cb(self, msg):

        self.load_problem_structure(msg.data)
        for name in self.agent_names:
            msg_out = MissionInfo()
            msg_out.agent = name
            if self.position[2] > 0:
                msg_out.position = self.position
            msg_out.status = "start"
            self.info_pub.publish(msg_out)
        # wait a bit to get all messages
        rospy.sleep(1)

        self.configure_problem_structure(msg.data)

        if self.mdvrp is not None:
            start = rospy.get_time()
            self.algorithm.optimize(self.mdvrp)
            self.alg_runtime = rospy.get_time() - start
            self.finish_mission()
        else:
            self.logger.warn("No mdvrp structure to optimize.")

    @abstractmethod
    def load_problem_structure(self, problem_id):
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

    def share_mdvrp(self, Id):

        tmp = self.mdvrp.setup_duration_matrix[Id]
        tmp = np.reshape(tmp, (tmp.size))
        self.setup_duration_pub.publish(tmp)

        tmp = self.mdvrp.setup_cost_matrix[Id]
        tmp = np.reshape(tmp, (tmp.size))
        self.setup_cost_pub.publish(tmp)

        tmp = self.mdvrp.duration_matrix[Id]
        np.reshape(tmp, (tmp.size))
        self.duration_pub.publish(tmp)

        tmp = self.mdvrp.demand_matrix[Id]
        np.reshape(tmp, (tmp.size))
        self.demand_pub.publish(tmp)

        tmp = self.mdvrp.quality_matrix[Id]
        np.reshape(tmp, (tmp.size))
        self.quality_pub.publish(tmp)

        # wait a bit to start (until all info is exchanged)
        rospy.sleep(1)

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
        self.setupDurationOffset = {}
        self.setupCostOffset = {}
        self.durationOffset = {}
        self.demandOffset = {}
        self.qualityOffset = {}

        self.robot_positions = {}
        self.robot_status = []
        self.position = [-1, -1, -1]
        # self.robots = []
        self.logger.info("Initialized mission structures")

    def mission_info_cb(self, msg):
        if msg.status == "start":
            self.robot_positions[msg.agent] = msg.position
            self.robot_status.append(0)
        elif msg.status == "done":
            labels = self.robot_positions.keys()
            self.robot_status[labels.index(msg.agent)] = 1
            if min(self.robot_status) == 1:
                self.algorithm.done = True

    def finished_cb(self, msg):
        msgOut = MissionInfo()
        msgOut.agent = msg.data
        msgOut.status = "done"
        self.info_pub.publish(msgOut)

    def setup_duration_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        n = sqrt(msg.data.size)
        while self.mdvrp is None:
            rospy.sleep(0.1)
        Id = self.mdvrp.vehicle_labels.index(sender)
        if Id not in self.setupDurationOffset:
            self.setupDurationOffset[Id] = 0
        o = self.setupDurationOffset[Id]
        self.mdvrp.setup_duration_matrix[Id +
                                         o] = np.reshape(msg.data, (int(n), int(n)))
        self.setupDurationOffset[Id] += 1

    def setup_cost_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        n = sqrt(msg.data.size)
        while self.mdvrp is None:
            rospy.sleep(0.1)
        Id = self.mdvrp.vehicle_labels.index(sender)
        if Id not in self.setupCostOffset:
            self.setupCostOffset[Id] = 0
        o = self.setupCostOffset[Id]
        self.mdvrp.setup_cost_matrix[Id +
                                     o] = np.reshape(msg.data, (int(n), int(n)))
        self.setupCostOffset[Id] += 1

    def duration_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        while self.mdvrp is None:
            rospy.sleep(0.1)
        Id = self.mdvrp.vehicle_labels.index(sender)
        if Id not in self.durationOffset:
            self.durationOffset[Id] = 0
        o = self.durationOffset[Id]
        self.mdvrp.duration_matrix[Id + o] = msg.data
        self.durationOffset[Id] += 1

    def demand_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        while self.mdvrp is None:
            rospy.sleep(0.1)
        Id = self.mdvrp.vehicle_labels.index(sender)
        if Id not in self.demandOffset:
            self.demandOffset[Id] = 0
        o = self.demandOffset[Id]
        self.mdvrp.demand_matrix[Id + o] = msg.data
        self.demandOffset[Id] += 1

    def quality_cb(self, msg):
        sender = msg._connection_header['callerid'].split('/')[1]
        while self.mdvrp is None:
            rospy.sleep(0.1)
        Id = self.mdvrp.vehicle_labels.index(sender)
        if Id not in self.qualityOffset:
            self.qualityOffset[Id] = 0
        o = self.qualityOffset[Id]
        self.mdvrp.quality_matrix[Id + o] = msg.data
        self.qualityOffset[Id] += 1
