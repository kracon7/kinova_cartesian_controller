import yaml
import rospy
from scipy.spatial.transform import Rotation as sci_rot
import numpy as np
from geometry_msgs import Transform
from kortex_driver.srv import *
from kortex_driver.msg import *

from math import pi
from math import sin, cos
import time

class VelocityController:
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name',"my_gen3")

        self.target_msg = None
        self.target_sub = rospy.Subscriber('cartesian_target', Transform, self.target_cb)

        self.pose_sub = rospy.Subscriber("/" + robot_name + "/base_feedback", 
                                         BaseCyclic_Feedback,
                                         self.pose_cb)
        current_base = rospy.wait_for_message("/" + robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.pose = kortex_base_to_transform(current_base.base)

        self.twist_cmd_pub = rospy.Publisher('my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)

    def target_cb(self, target_msg):
        self.target_msg = target_msg

    def pose_cb(self, pose_msg):
        base = pose_msg.base
        self.pose = kortex_base_to_transform(base)

    def calculate_twist(self):
        '''
        Compare target and current pose and compute twist command
        '''
        return twist
    
    def publish_twist_cmd(self, twist: np.ndarray):
        twsit_cmd = write_twist_cmd(twist)
        self.twist_cmd_pub.publish(twsit_cmd)

def kortex_base_to_transform(base: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3,3] = np.array([base.tool_pose_x, 
                                    base.tool_pose_y, 
                                    base.tool_pose_z])
    T[:3,:3] = sci_rot.from_euler('zyx', 
                                  [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z],
                                  degrees=True)
    return T

def transform_to_kortex_base(T: np.ndarray) -> np.ndarray:
    base = [T[0,3], T[1,3], T[2,3]]
    euler = sci_rot.from_matrix(T[:3,:3]).as_euler('zyx', degrees=True)
    base += [euler[0], euler[1], euler[2]]
    return np.array(base)


def write_twist_cmd(ref_frame=0, twist=[0.,0.,0.,0.,0.,0.], duration=0):
    twist_cmd = TwistCommand()
    twist_cmd.reference_frame = ref_frame
    twist_cmd.twist.linear_x = twist[0]
    twist_cmd.twist.linear_y = twist[1]
    twist_cmd.twist.linear_z = twist[2]
    twist_cmd.twist.angular_x = twist[3]
    twist_cmd.twist.angular_y = twist[4]
    twist_cmd.twist.angular_z = twist[5]
    twist_cmd.duration = duration
    return twist_cmd

        
if __name__ == "__main__":
    rospy.init_node('kortex_velocity_controller')
    rate=rospy.Rate(200)

    velocity_controller = VelocityController()

    while not rospy.is_shutdown():
        twist = velocity_controller.calculate_twist()
        velocity_controller.publish_twist_cmd(twist)
        rate.sleep()