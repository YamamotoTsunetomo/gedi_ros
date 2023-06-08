#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped


HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
car_cmd = f'/{HOST}/joy_mapper_node/car_cmd'
velocity = f'/{HOST}/kinematics_node/velocity'
wheels_cmd = f'/{HOST}/wheels_driver_node/wheels_cmd'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'

class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # testing for all Twist2DStamped, WheelsCmdStamped


        # CMD
        self.cmd_pub = rospy.Publisher(
            cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.cmd_sub = rospy.Subscriber(
            cmd,
            Twist2DStamped,
            self.cmd_cb,
            queue_size=1
        )

        # CMD_PUB
        self.car_cmd_pub = rospy.Publisher(
            car_cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.car_cmd_sub = rospy.Subscriber(
            car_cmd,
            Twist2DStamped,
            self.car_cmd_cb,
            queue_size=1
        )

        # VELOCITY
        self.velocity_pub = rospy.Publisher(
            velocity,
            Twist2DStamped,
            queue_size=1
        )

        self.velocity_sub = rospy.Subscriber(
            velocity,
            Twist2DStamped,
            self.velocity_cb,
            queue_size=1
        )

        # WHEELS_CMD
        self.wheels_cmd_pub = rospy.Publisher(
            wheels_cmd,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_sub = rospy.Subscriber(
            wheels_cmd,
            WheelsCmdStamped,
            self.wheels_cmd_cb,
            queue_size=1
        )

        # WHEELS_CMD_EXECUTED
        self.wheels_cmd_executed_pub = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_executed_sub = rospy.Subscriber(
            wheels_cmd_executed,
            WheelsCmdStamped,
            self.wheels_cmd_executed_cb,
            queue_size=1
        )

    def cmd_cb(self, twist):
        rospy.loginfo(f'cmd received {type(twist)}')

    def car_cmd_cb(self, twist):
        rospy.loginfo(f'cmd received {type(twist)}')

    def velocity_cb(self, twist):
        rospy.loginfo(f'cmd received {type(twist)}')
        

    def wheels_cmd_cb(self, wheels):
        rospy.loginfo(f'cmd received {type(wheels)}')

    def wheels_cmd_executed_cb(self, wheels):
        rospy.loginfo(f'cmd received {type(wheels)}')

    def run(self):
        wheel = WheelsCmdStamped()
        wheel.vel_right = 1
        wheel.vel_left = 1

        twist = Twist2DStamped()
        twist.v = 40
        twist.omega = 10

        while not rospy.is_shutdown():
            self.wheels_cmd_pub.publish(wheel)
            self.wheels_cmd_executed_pub.publish(wheel)
            self.cmd_pub.publish(twist)


if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()
