""" experiment.py: entry-point for Aion rover experiment. """

import time
import numpy as np
from typing import List
import nptyping as npt
import builtins
from dasc_robots.robot import Robot
from dasc_robots.ros_functions import *
from core.agent import Agent
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformations import euler_from_quaternion


def get_robot_states(robots: List[Robot]) -> npt.NDArray:
    """Given a list of Robot objects, get all states and return as one numpy array.

    ARGUMENTS
    ---------
    robots: list of Robot objects

    RETURNS
    -------
    states: Nxn array of N robots containing n states

    """
    states = np.zeros((len(robots), 5))
    for rr, robot in enumerate(robots):
        pos = robot.get_world_position()
        vel = robot.get_world_velocity()
        phi = euler_from_quaternion(robot.get_body_quaternion())[2]

        print("pos: {}".format(pos), "vel: {}".format(vel), "phi: {}".format(phi))

        states[rr, :] = np.array([pos[0], pos[1], phi, np.linalg.norm(vel), 0.0])

    return states




class MinimalPublisher(Node):

    def __init__(self,
                 robots: List[Robot],
                 agents: List[Agent]):
        super().__init__('minimal_publisher')
        self.robots = robots
        self.agents = agents
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer_period = 0.05  # seconds

        # Start Timer Callbacks
        [self.create_timer(self.timer_period, self.timer_callback_wrapper(rr, robot)) for rr, robot in enumerate(robots)]

        # Settings
        nTimesteps = 100000
        nStates = 5
        nControls = 2

        # Logging variables
        self.i = 0
        self.z = np.zeros((nTimesteps, len(robots), nStates))
        self.u = np.zeros((nTimesteps, len(robots), nControls))

    def timer_callback_wrapper(self,
                               rr: int,
                               robot: Robot):
        """"""

        def timer_callback():
            self.i += 1
            agent = self.agents[rr]

            z = get_robot_states(self.robots)
            self.z[self.i] = z

            code, status = agent.compute_control(z)

            # Compute velocity command control inputs
            omg = agent.controller.u[0]
            acc = agent.controller.u[1]
            vel = z[rr, 3] + self.timer_period * acc

            deadband = 0.25
            if abs(vel) < deadband and acc > 0:
                vel = deadband * np.sign(vel)

            vel = np.clip(vel, -1, 1)

            if rr > -1:
                omg = 0.0
                vel = 0.0

            else:
                
                print("Omega: {:.2f}!".format(omg))
                print("Veloc: {:.2f}!".format(vel))

            cmd = np.array([0, vel, 0, 0, omg])
            robot.command_velocity(cmd)

        return timer_callback


def _experiment(tf: float,
                dt: float,
                vehicle: str,
                level: str,
                situation: str) -> bool:
    """Simulates the system specified by config.py for tf seconds at a frequency of dt.

    ARGUMENTS
        tf: final time (in sec)
        dt: timestep length (in sec)
        vehicle: the vehicle to be simulated
        level: the control level (i.e. kinematic, dynamic, etc.)
        situation: i.e. intersection_old, intersection, etc.

    RETURNS
        None

    """
    # Program-wide specifications
    builtins.PROBLEM_CONFIG = {'vehicle': vehicle,
                               'control_level': level,
                               'situation': situation,
                               'system_model': 'deterministic'}

    if vehicle == 'bicycle':
        from bicycle import nAgents, nStates, z0, decentralized_agents as agents

    broken = False
    nTimesteps = int((tf - 0.0) / dt) + 1

    # Simulation setup
    z = np.zeros((nTimesteps, nAgents, nStates))
    complete = np.zeros((nAgents,))

    # ROS Setup
    rclpy.init(args=None)
    ros_init("C-CBF Rover Experiment")

    # Create Robots
    rover_ids = [3, 5, 7]
    robots = [Robot("rover{}".format(rr), rr) for rr in rover_ids]

    # Start ROS Nodes
    threads = start_ros_nodes(robots)

    # Initialize Robots
    for robot in robots:
        robot.init()

     # Sleep to connect
    print("Connecting...")
    time.sleep(5)

    # Arm Robots
    for robot in robots:
        robot.set_command_mode('velocity')

    for ii in range(100):
        for rr, robot in enumerate(robots):
            robot.command_velocity(np.zeros(5,))
            time.sleep(0.01)

    for rr, robot in enumerate(robots):
        robot.cmd_offboard_mode()
        robot.arm()
        print("Robot{} Armed".format(rover_ids[rr]))
    
    # Set up publishers
    minimal_publisher = MinimalPublisher(robots, agents)
    rclpy.spin(minimal_publisher)

    # Destroy
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    # Save data
    for aa, agent in enumerate(decentralized_agents):
        agent.save_data(aa)

    success = not broken

    return success


def experiment():
    end_time = 40.0
    timestep = 0.05
    veh = 'bicycle'
    lev = 'dynamic'
    sit = 'dasclab'

    success = _experiment(end_time, timestep, veh, lev, sit)