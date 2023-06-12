import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from pynput import keyboard

class JoystickNode(DTROS):
    def __init__(self, node_name):
        super(JoystickNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.key_pub = rospy.Publisher(
            'control_keys',
            String,
            queue_size=1
        )
        self.listener = keyboard.Listener(on_press=self.on_press)

    def on_press(self, key):
        try:
            k = key.char
            rospy.loginfo(f'sending control {k}')
            self.key_pub.publish(k)
        except:
            rospy.loginfo(f'unsupported control key {key}')

if __name__ == '__main__':
    node = JoystickNode(node_name='node_name')
    rospy.spin()