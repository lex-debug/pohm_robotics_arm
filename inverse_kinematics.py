# ROS Client Library for Python
import rclpy

# Handles the creation of nodes
from rclpy.node import Node

# ROS Message API
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

# ROS Utils API
import pkg_resources

import numpy as np
import pickle
import sys
import math

sys.path.insert(1, './anfis/')
import anfis
# from .anfis.anfis import *

class InverseKinematicsNode(Node):
    """
    Class constructor to set up the node
    """
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('ik_anfis_node')

        self.logger_ = self.get_logger()

        self.logger_.info('Starting Inverse Kinematics node...')

        # load the pretrained anfis network
        self.data_file_path1 = pkg_resources.resource_filename('pohm_robotics_arm', 'data/j1Anf.pickle')
        with open(self.data_file_path1, 'rb') as f:
            self.j1Anf = pickle.load(f)

        self.data_file_path2 = pkg_resources.resource_filename('pohm_robotics_arm', 'data/j2Anf.pickle')
        with open(self.data_file_path2, 'rb') as f:
            self.j2Anf = pickle.load(f)

        # Create a subscriber which will subscribe to the coor topic published
        # by the Intel Realsense
        self.logger_.info('Creating FFB Pose subscriber...')
        self.ffb_position_subscriber_  = self.create_subscription(Pose, 'coor', self.ffb_position_subscriber_callback, 10)
        self.ffb_position_subscriber_ # prevent unused variable warning
        self.logger_.info('Done')

        # Create a subscriber which will subscribe to the joint_angles_feedback topic published
        # by the STM32F4 Discovery Board in order to calculate the transformation of camera
        # with respect to the base
        self.logger_.info('Creating Joint Angles Feedback subscriber...')
        self.joint_angles_fb_subscriber = self.create_subscription(Float32MultiArray, 'joint_angles_feedback', self.joint_angles_fb_subscriber_callback, 10)
        self.joint_angles_fb_subscriber
        self.logger_.info('Done')

        # Create the publisher. This publisher will publish an array of Float32 data
        # to the joint topic. The queue size is 10 messages.
        self.logger_.info('Creating Joint Angles Command publisher...')
        self.joint_angle_publisher_ = self.create_publisher(Float32MultiArray, 'joint', 10)
        self.logger_.info('Done')

        # Parameters required for calculating the camera transformation
        # This parameters are only valid for the MRANTI Robot
        self.l0 = 0.658
        self.l0_fixed_angle = 1.9197
        self.l_camera = 0.38
        self.l_camera_fixed_angle = 1.0472 # the camera has an offset angle with the ground level

        self.c1 = self.l0 * math.cos(self.l0_fixed_angle)
        self.s1 = self.l0 * math.sin(self.l0_fixed_angle)

        self.x_c = 0.0
        self.y_c = 0.0
        self.theta_c = 0.0

        # Parameters of Robotics Arm
        # These are obtained from the DataGeneration_MRANTI.ipynb
        self.X_MIN = 0.5142
        self.X_MAX = 1.2251
        self.Y_MIN = 0.6184
        self.Y_MAX = 1.7292
        self.cutting_offset = 0.1

        # Flag
        self.fb_ready = 0

    def publish_joint_angles(self, j0Angle, j1Angle, j2X):
        # Create a Float Array msg
        msg = Float32MultiArray()

        # Fill up the Float Array with joint angles
        msg.data = [j0Angle, j1Angle, j2X, 50.0]

        self.joint_angle_publisher_.publish(msg)

    def ffb_position_subscriber_callback(self, msg):
        # The FFB position in terms of x, y and z will be fed into the anfis to
        # predict the joint angles
        self.logger_.info('Getting camera transformation with respect to the base')
        if self.fb_ready == 0:
            self.logger_.error('Cannot calculate the camera transformation. Please make sure the joint angle feedbacks have been published')
        else:
            self.logger_.info('Camera Transformation: [{:.4f}, {:.4f}, {:.4f}]'.format(self.x_c, self.y_c, self.theta_c))
            self.logger_.info('Getting FFB position...')
            x = msg.position.z * math.cos(self.theta_c) + self.x_c
            y = -1 * (msg.position.y * math.cos(self.theta_c)) + self.y_c
            self.logger_.info('FFB position: ({:.4f}, {:.4f})'.format(x, y))
            if (x < self.X_MIN or x > self.X_MAX) or (y < self.Y_MIN or y > self.Y_MAX):
                self.logger_.error('The position of the ffb is out of reach by the robotics arm, please move the robotics arm closer or further.')
            else:
                # normalize the x and y coordinate before putting into the fuzzy inference system
                x = ((x - self.X_MIN) / (self.X_MAX - self.X_MIN)) * 20.0 - 10.0
                y = ((y - self.Y_MIN) / (self.Y_MAX - self.Y_MIN)) * 20.0 - 10.0
                ffb_position_coordinate = np.array([[x, y]])
                # The reason that the cutting offset is added to the x instead of z is 
                # because that the chainsaw is placed at an offset distance beside the FFB
                j0Angle = math.atan2((msg.position.x + self.cutting_offset), msg.position.z)
                j1Angle = anfis.predict(self.j1Anf, ffb_position_coordinate)[0][0]
                j2X = anfis.predict(self.j2Anf, ffb_position_coordinate)[0][0]

                # Convert to degree
                j0Angle = j0Angle * (180.0/math.pi)
                j1Angle = j1Angle * (180.0/math.pi)
                self.logger_.info('Publishing to the joint with the following command')
                self.logger_.info('j0Angle: {:.4f}'.format(j0Angle))
                self.logger_.info('j1Angle: {:.4f}'.format(j1Angle))
                self.logger_.info('j2X: {:.4f}'.format(j2X))
                self.publish_joint_angles(j0Angle, j1Angle, j2X)

            self.fb_ready = 0

    def joint_angles_fb_subscriber_callback(self, msg):
        # Since the joint angles published by the STM32F4 Discovery is in degree,
        # they need to be converted into radian
        self.fb_ready = 1
        j1Angle_fb = msg.data[1]
        j2X_fb = msg.data[2]
        j1Angle_fb = j1Angle_fb * math.pi / 180.0
        camera_angle_prime = j1Angle_fb + self.l_camera_fixed_angle

        self.c2 = (j2X_fb - 0.3) * math.cos(j1Angle_fb) # the camera is 0.3m away from the tip of the robotics arm
        self.c3 = self.l_camera * math.cos(camera_angle_prime)

        self.s2 = (j2X_fb - 0.3) * math.sin(j1Angle_fb)
        self.s3 = self.l_camera * math.sin(camera_angle_prime)

        self.x_c = self.c1 + self.c2 + self.c3
        self.y_c = self.s1 + self.s2 + self.s3 
        self.theta_c = camera_angle_prime - (math.pi / 2) # not necessary, but need to recheck again

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    inverse_kinematics_node = InverseKinematicsNode()

    # Spin the node so the callback function is called.
    rclpy.spin(inverse_kinematics_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inverse_kinematics_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()