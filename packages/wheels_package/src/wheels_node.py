#!/usr/bin/env python3

import os
import rospy
import json
import yaml
import time

from std_srvs.srv import EmptyResponse, Empty
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import String, Bool


HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
velocity = f'/{HOST}/kinematics_node/velocity'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'
car_cmd = f'/{HOST}/joy_mapper_node/car_cmd'

class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # self.vehicle_name = rospy.get_namespace().strip("/")

        # self.read_calibration_params()

        # self._k = DTParam("~k", param_type=ParamType.FLOAT, min_value=0.1, max_value=0.5)
        # parameters
        # self._gain = DTParam("~gain", param_type=ParamType.FLOAT, min_value=0.1, max_value=1.0)
        # self._trim = DTParam("~trim", param_type=ParamType.FLOAT, min_value=0.1, max_value=1.0)
        # self._limit = DTParam("~limit", param_type=ParamType.FLOAT, min_value=0.1, max_value=1.0)
        # self._baseline = DTParam("~baseline", param_type=ParamType.FLOAT, min_value=0.05, max_value=0.2)
        # self._radius = DTParam("~radius", param_type=ParamType.FLOAT, min_value=0.01, max_value=0.1)
        # self._v_max = DTParam("~v_max", param_type=ParamType.FLOAT, min_value=0.01, max_value=2.0)
        # self._omega_max = DTParam("~omega_max", param_type=ParamType.FLOAT, min_value=1.0, max_value=10.0)

        # self.is_obstacle_detected = False
        self.wh = self.get_wheel(0.4, 0.4)
        self.tw = self.get_twist(0.4, 0)

        self.pub_cmd = rospy.Publisher(
            cmd,
            Twist2DStamped,
            queue_size=1,
        )

        self.pub_velocity = rospy.Publisher(
            velocity,
            Twist2DStamped,
            queue_size=1,
        )

        self.pub_wheels_cmd = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.sub_car_cmd = rospy.Subscriber(
            car_cmd,
            Twist2DStamped,
            self.car_cmd_cb
        )

        self.pub_car_cmd = rospy.Publisher(
            car_cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.sub_detected = rospy.Subscriber(
            'obstacle_detected',
            Bool,
            self.det_cb,
            queue_size=1
        )

        self.detect = False

    def stop(self):
        wh = self.get_wheel(0,0)
        tw = self.get_twist(0,0)
        self.move(wheel=wh, twist=tw)

    def det_cb(self, b):
        rospy.loginfo(f'detect: {b}')
        self.detect = b.data
    
    def get_motion(self):
        return [ {            
                "Twist": self.get_twist(v=0.0, omega=8.300000190734863),
                "Wheel": self.get_wheel(left=-0.4661076068878174, right=0.46564173698425293),
                "TIMEDIFF": 0.196752
            },
            
            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 1.275982
            },
            
            {
                "Twist": self.get_twist(v=0.4099999964237213, omega=0.0),
                "Wheel": self.get_wheel(left=0.47776031494140625, right=0.4772827923297882),
                "TIMEDIFF": 0.933086
            },

            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.704729
            },
            
            {
                "Twist": self.get_twist(v=0.0, omega=-8.300000190734863),
                "Wheel": self.get_wheel(left=0.4661076068878174, right=-0.46564173698425293),
                "TIMEDIFF": 0.156581
            },

            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.771822
            },

            {
                "Twist": self.get_twist(v=0.4099999964237213, omega=0.0),
                "Wheel": self.get_wheel(left=0.47776031494140625, right=0.4772827923297882),
                "TIMEDIFF": 0.536066
            },

            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.463154
            },

            {
                "Twist": self.get_twist(v=0.0, omega=-8.300000190734863),
                "Wheel": self.get_wheel(left=0.4661076068878174, right=-0.46564173698425293),
                "TIMEDIFF": 0.161637
            },

            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.602932
            },

            {
                "Twist": self.get_twist(v=0.4099999964237213, omega=0.0),
                "Wheel": self.get_wheel(left=0.47776031494140625, right=0.4772827923297882),
                "TIMEDIFF": 0.634661
            },

            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.7356
            },

            {
                "Twist": self.get_twist(v=0.4099999964237213, omega=0.0),
                "Wheel": self.get_wheel(left=0.47776031494140625, right=0.4772827923297882),
                "TIMEDIFF": 0.167894
            },

            {

                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.256256
            },

            {
                "Twist": self.get_twist(v=0.4099999964237213, omega=0.0),
                "Wheel": self.get_wheel(left=0.47776031494140625, right=0.4772827923297882),
                "TIMEDIFF": 0.296109
            },

            {

                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.367904
            },

            {

                "Twist": self.get_twist(v=0.0, omega=8.300000190734863),
                "Wheel": self.get_wheel(left=-0.4661076068878174, right=0.46564173698425293),
                "TIMEDIFF": 0.190757},

            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.365342},

            {
                "Twist": self.get_twist(v=0.0, omega=8.300000190734863),
                "Wheel": self.get_wheel(left=-0.4661076068878174, right=0.46564173698425293),
                "TIMEDIFF": 0.095035},
            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.449475},

            {

                "Twist": self.get_twist(v=0.4099999964237213, omega=0.0),
                "Wheel": self.get_wheel(left=0.47776031494140625, right=0.4772827923297882),
                "TIMEDIFF": 0.877571},
            {
                "Twist": self.get_twist(v=0.0, omega=0.0),
                "Wheel": self.get_wheel(left=0.0, right=0.0),
                "TIMEDIFF": 0.877571
                }
                    ]
        
    def run(self):
        rospy.sleep(2)
        # motions = self.get_motion()
        # for motion in motions:
        #     tw = motion['Twist']
        #     wh = motion['Wheel']
        #     pause = motion['TIMEDIFF']

        #     self.move(wh, tw)
        #     rospy.R
        # ate(1 / pause).sleep()
        
        wh = self.get_wheel(0.2, 0.2)
        tw = self.get_twist(0.25, 0)
        self.move(wh, tw)

        while not rospy.is_shutdown():
            if self.detect:
                self.stop()
                rospy.sleep(1)
                self.law_abiding_citizen()
                rospy.sleep(1)
                self.dejavu()
                self.stop()
                break
        # self.test_move()
        self.move(wh, tw)
        rospy.sleep(2)
        self.stop()


    def get_wheel(self, left, right):
        wheel = WheelsCmdStamped()
        wheel.vel_left = left
        wheel.vel_right = right
        return wheel
    
    def dejavu(self):
        tw = self.get_twist(0.25, -7)
        wh = self.get_wheel(0.1, 0.2)
        self.move(wh, tw)

        rospy.sleep(3)
        
        # tw = self.get_twist(0.25, 0)
        # wh = self.get_wheel(0.2, 0.2)
        # self.move(wh, tw)

        # rospy.sleep(1)

        # tw = self.get_twist(0.25, 8)
        # wh = self.get_wheel(0.4, 0.2)
        # self.move(wh, tw)

        # rospy.sleep)

    def law_abiding_citizen(self):
        # left
        tw = self.get_twist(0.3, 5)
        wh = self.get_wheel(0.1, 0.7)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()

        # right 
        tw = self.get_twist(0.4, -5)
        wh = self.get_wheel(0.7, 0.1)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()

        # right        
        tw = self.get_twist(0.3, -5)
        wh = self.get_wheel(0.7, 0.1)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()

        # left        
        tw = self.get_twist(0.45, 5)
        wh = self.get_wheel(0.1, 0.7)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()



    def get_twist(self, v, omega):
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        return twist

    def move(self, wheel, twist):
        self.pub_wheels_cmd.publish(wheel)
        self.pub_cmd.publish(twist)


    def test_move(self):
        moves = [
            # forward
            [
                self.get_twist(0.25,0), 
                self.get_wheel(0.2, 0.2)
            ],

            # omega > 0, left < right
            [
                self.get_twist(0.25,2), 
                self.get_wheel(0.2, 0.7)
            ],

            # omega < 0, left > right
            [
                self.get_twist(0.25,-2), 
                self.get_wheel(0.7, 0.2)
            ],

            # omega > 0, left = right
            [
                self.get_twist(0.25,2), 
                self.get_wheel(0.2, 0.4)
            ],

            # omega < 0 left = right
            [
                self.get_twist(0.25, -2), 
                self.get_wheel(0.4, 0.2)
            ],
        ]

        for move in moves:
            tw = move[0]
            wh = move[1]
            self.move(wh, tw)
            rospy.Rate(1).sleep()
            self.stop()
            rospy.Rate(1).sleep()

    
    def get_current_configuration(self):
        pass
        # return {
            # "gain": rospy.get_param("~gain",self._gain),
            # "trim": rospy.get_param("~trim",self._trim),
            # "baseline": rospy.get_param("~baseline",self._baseline),
            # "radius": rospy.get_param("~radius",self._radius),
            # "k": rospy.get_param("~k",self._k),
            # "limit": rospy.get_param("~limit",self._limit),
            # "v_max": rospy.get_param("~v_max",self._v_max),
            # "omega_max": rospy.get_param("~omega_max",self._omega_max),
        # }

    def read_calibration_params(self):
        pass
        # file_name = self.get_calibration_filepath(self.vehicle_name)

        # if not os.path.isfile(file_name):
        #     self.logwarn("Kinematics calibration %s not found! Using default" % file_name)
        # else:
        #     with open(file_name, 'r') as in_file:
        #         try:
        #             yaml_dict = yaml.load(in_file, Loader=yaml.FullLoader)
        #         except yaml.YAMLError as e:
        #             self.logfatal(f'Yaml syntax error: {e}')
        #             rospy.signal_shutdown()
        #             return
        #     if yaml_dict is None:
        #         return
        #     for param_name in self.get_current_configuration().keys():
        #         param_value = yaml_dict.get(param_name)
        #         if param_name is not None and param_value is not None:
        #             rospy.set_param("~" + param_name, param_value)
            
    @staticmethod
    def get_calibration_filepath(name):
        pass
        # cali_folder = '/data/config/calibrations/kinematics/'
        # return cali_folder + name + '.yaml'
    
    @staticmethod
    def trim(value, low, high):
        # return max(min(value, high), low)
        pass
    
    def car_cmd_cb(self, twist):
        twist.v = self.trim(
            value=twist.v,
            low=-self._v_max.value, 
            high=self._v_max.value
        )

        twist.omega = self.trim(
            value=twist.omega,
            low=-self._omega_max.value,
            high=self._omega_max.value
        )

        # setting motor constants
        k_r = k_l = self._k

        # adjust k by gain and trim
        k_r_inv = (self._gain.value + self._trim.value) / k_r
        k_l_inv = (self._gain.value - self._trim.value) / k_l

        omega_r = (twist.v + 0.5 * twist.omega * self._baseline.value) / self._radius.value
        omega_l = (twist.v - 0.5 * twist.omega * self._baseline.value) / self._radius.value

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # limiting output
        u_r_lim = self.trim(
            value=u_r,
            low=-self._limit.value,
            high=self._limit.value
        )
        
        u_l_lim = self.trim(
            value=u_l,
            low=-self._limit.value,
            high=self._limit.value
        )

        # publishing wheel
        wh = self.get_wheel(l=u_l_lim, r=u_r_lim)
        wh.header.stamp = twist.header.stamp
        self.pub_wheels_cmd.publish(wh)

        # conversion from motor duty to motor rotation rate
        omega_r = wh.vel_right / k_r_inv
        omega_l = wh.vel_left / k_l_inv

        # velocity calculation
        v = (self._radius.value * omega_r + self._radius.value * omega_l) / 2.0
        omega = (self._radius.value * omega_r + self._radius.value * omega_l) / self._baseline.value 

        # publishing velocity
        tw = self.get_twist(v=v, omega=omega)
        tw.header = twist.header
        self.pub_velocity.publish(tw)

    def obstacle_dt_cb(self, is_detected):
        # print(f'duckiebot: {is_detected.data}')
        self.is_obstacle_detected = is_detected.data
        print(f'obstacle detected:{self.is_obstacle_detected}')

if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()
