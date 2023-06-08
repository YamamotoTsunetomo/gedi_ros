#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped


class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # self.sub = rospy.Subscriber(
        #     '/walle/kinematics_node/velocity',
        #     Twist2DStamped,
        #     self.callback,
        #     queue_size=1
        # )

        self._starter = rospy.Publisher(
            '/gedi/joy_mapper_node/joystick_override',
            BoolStamped,
            queue_size=1
        )

        msg = BoolStamped()
        msg.data = False
        self._starter.publish(msg)

        self.wh_sub = rospy.Subscriber(
            'walle/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            self.wh_callback,
            queue_size=1
        )

        self.velocity_sub = rospy.Subscriber(
            'walle/kinematics_node/velocity',
            Twist2DStamped,
            self.velocity_callback,
            queue_size=1
        )

        self.wh_pub = rospy.Publisher(
            'walle/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1
        )

        self.velocity_pub = rospy.Publisher(
            'walle/kinematics_node/velocity',
            Twist2DStamped,
            queue_size=1
        )

    # def callback(self, msg): # type(msg) = Twist2DStamped
    #     print(f'received of type {type(msg)}')
    #
    #     msg_wheels_cmd = WheelsCmdStamped()
    #     msg_wheels_cmd.header.stamp = msg.header.stamp
    #     # commenting out calibration bullshit
    #     msg_wheels_cmd.vel_right = 100 #u_r_limited
    #     msg_wheels_cmd.vel_left = 100 #u_l_limited
    #     self.pub.publish(msg_wheels_cmd)

    def wh_callback(self, wh):
        rospy.loginfo(f'received wheels info with vel_left={wh.vel_left}, vel_right={wh.vel_right}')

    def velocity_callback(self, vel):
        rospy.loginfo(f'received Twist2DStamped with velocity={vel.v}, omega={vel.omega}')

    def run(self):
        msg = WheelsCmdStamped()
        msg.vel_right = 1
        msg.vel_left = 0

        vel = Twist2DStamped()
        vel.v = 40
        vel.omega = 10
        while not rospy.is_shutdown():
            rospy.loginfo('publishing the SHIT!')
            self.wh_pub.publish(msg)
            self.velocity_pub.publish(vel)


if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()
