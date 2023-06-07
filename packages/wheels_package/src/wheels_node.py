#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name,node_type=NodeType.CONTROL)
        self.sub = rospy.Subscriber(
            '/walle/kinematics_node/velocity',
            Twist2DStamped,
            self.callback,
            queue_size=1
        )
        self.pub = rospy.Publisher(
            'walle/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1
        )
    
    def callback(self, msg): # type(msg) = Twist2DStamped
        print(f'received of type {type(msg)}')

        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg.header.stamp
        # commenting out calibration bullshit
        msg_wheels_cmd.vel_right = 100 #u_r_limited
        msg_wheels_cmd.vel_left = 100 #u_l_limited
        self.pub.publish(msg_wheels_cmd)

    def run(self):
        msg = WheelsCmdStamped()
        msg.right = 100
        msg.left = 100
        while not rospy.is_shutdown():
            rospy.loginfo('publishing the SHIT!')
            self.pub.publish(msg)

if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()

