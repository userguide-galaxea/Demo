#!/usr/bin/env python

import rospy
import sys
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from hdas_msg.msg import motor_control
import time
from std_msgs.msg import Bool

class JointStateSender:
    def __init__(self):
        rospy.init_node('joint_state_sender', anonymous=True)
        self.left_joint_state_pub = rospy.Publisher('/motion_target/target_joint_state_arm_left', JointState, queue_size=10)
        self.right_joint_state_pub = rospy.Publisher('/motion_target/target_joint_state_arm_right', JointState, queue_size=10)
        self.torso_joint_state_pub = rospy.Publisher('/torso_joint_target_position_', JointState, queue_size=10)
        self.torso_joint_state_pub_real = rospy.Publisher('/motion_target/target_joint_state_torso', JointState, queue_size=10)
        self.chassis_command_pub = rospy.Publisher('/motion_target/target_speed_chassis',Twist, queue_size=10)
        self.acc_limit_pub = rospy.Publisher('/motion_target/chassis_acc_limit',Twist,queue_size=10)
        self.breaking_mode_pub = rospy.Publisher('/motion_target/brake_mode',Bool,queue_size=10)
        self.command = None

    def send_joint_state(self, position_left, position_right, position_torso):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub.publish(torso_joint_state)

    def send_joint_state_real(self, position_left, position_right, position_torso):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)
    def send_vel_limit(self, vel_limit_left, vel_limit_right, vel_limit_torso):
        left_joint_state = JointState()
        left_joint_state.velocity = vel_limit_left
        right_joint_state = JointState()
        right_joint_state.velocity = vel_limit_right
        torso_joint_state = JointState()
        torso_joint_state.velocity = vel_limit_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)

    def send_chassis(self, chassis_command):
        chassis_command_msg = Twist()
        chassis_command_msg.linear.x = chassis_command[0]
        chassis_command_msg.linear.y = chassis_command[1]
        chassis_command_msg.angular.z = chassis_command[2]
        self.chassis_command_pub.publish(chassis_command_msg)
    def send_acc_limit(self,acc_limit):
        acc_limit_msg = Twist()
        acc_limit_msg.linear.x = acc_limit[0]
        acc_limit_msg.linear.y = acc_limit[1]
        acc_limit_msg.angular.z = acc_limit[2]
        self.acc_limit_pub.publish(acc_limit_msg)
    def send_breaking_mode(self,breaking_mode_signal):
        breaking_mode_msg = Bool()
        breaking_mode_msg.data = breaking_mode_signal[0]
        self.breaking_mode_pub.publish(breaking_mode_msg)

    def input_thread(self):
        while True:
            print('1: arm_test1')
            print('2: arm_test2')
            print('3: arm_test3')
            print('4: torso_test1')
            print('5: torso_test2')
            print('q: quit brake mode')
            self.command = input("Press '1' or '2' or '3' or '4' or '5' or 'q': ")

    def run(self):
        input_thread = threading.Thread(target=self.input_thread)
        input_thread.daemon = True
        
        #rospy.loginfo("Press '1' to send the first JointState, '2' for the second.")
        input_thread.start()
        try:

            while not rospy.is_shutdown():
                try:
                    command = self.command
                    self.command = None
                    if command == '1': # hand up test, end up with standing
                        # 按下2时发送第二个JointState
                        self.send_vel_limit([1.6,1.6,1.6,4,4,4],[1.6,1.6,1.6,4,4,4], [1,1.6,1.4,1.4])
                        self.send_breaking_mode([1])
                        self.send_joint_state_real([-1.56,0,0,0,0,0],[1.56,0,0,0,0,0],[0,0,0,0])
                        time.sleep(0.5)  # 防止重复发送
                        self.send_joint_state([1.56,2.94,-2.54,0,0,0],[-1.56,2.94,-2.54,0,0,0],[0,0,0,0])
                        time.sleep(0.5)  # 防止重复发送
                    elif command == '2': # kongfu standing
                        self.send_breaking_mode([1])
                        self.send_vel_limit([1.6,1.6,1.6,4,4,4],[1.6,1.6,1.6,4,4,4], [1,1.6,1.4,1.4])
                        self.send_joint_state_real([0.0,0.0,0.0,0,0,0],[0.0,0.0,0.0,0,0,0],[0.0, 0.0, 0.0, 0.0])
                        time.sleep(1)
                    elif command == '3': # hand dance
                        self.send_breaking_mode([1])
                        j5_bias=0.00 #left arm
                        j1_bias=0.00
                        # self.send_vel_limit([3,3,3,5,5,5],[3,3,3,5,5,5], [0.5,0.79,0.7,1.0])
                        self.send_vel_limit([2.4,2.4,2.4,4,4,4],[2.4,2.4,2.4,4,4,4], [0.5,0.79,0.7,1.0])
                        self.send_joint_state_real([1.5703,2.94,-2.54,0,0,0],[-1.5703,2.94,-2.54,0,0,0],[0.0,0,0,0])
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([1.53,1.61,-2.79,0,0,0],[-1.53,1.61,-2.79,0,0,0],[0,0,0,0])
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([-0.23,2.94,-2.54,0,0,0],[0.23,2.94,-2.54,0,0,0],[0.,0.,0.,0.])
                        time.sleep(2)  # 防止重复发送
                        self.send_joint_state_real([-1.5703,1.98,-1.21,-1.5703,0.95,0.13],[1.5703,1.98,-1.21,-1.5703,0.95,0.13],[0.0,0,0,0] )
                        time.sleep(3)  # 防止重复发送
                        self.send_joint_state_real([1.5703,2.94,-2.54,0,0,0],[-1.5703,2.94,-2.54,0,0,0],[0.0,0,0,0])
                        time.sleep(5)  # 防止重复发送
                    elif command == '4': # on your knees  
                        self.send_breaking_mode([1])     
                        self.send_vel_limit([1.6,1.6,1.6,4,4,4],[1.6,1.6,1.6,4,4,4], [1,1.6,1.4,1.4])      
                        self.send_joint_state_real([0,0,0,0,0,0],[0,0,0,0,0,0],[1.74, -2.70, -0.96, 0.0])
                        time.sleep(1)
                    elif command == '5': # t4 test
                        self.send_breaking_mode([1])
                        self.send_vel_limit([1.6,1.6,1.6,4,4,4],[1.6,1.6,1.6,4,4,4], [1,1.6,1.4,1.4])
                        self.send_joint_state_real([1.56,2.94,-2.54,0,0,0],[-1.56,2.94,-2.54,0,0,0],[0,0,0,0])
                        time.sleep(3)
                        self.send_joint_state_real([1.56,2.94,-2.54,0,0,0],[-1.56,2.94,-2.54,0,0,0],[0,0,0,0.75])
                        time.sleep(2)
                        self.send_joint_state_real([1.56,2.94,-2.54,0,0,0],[-1.56,2.94,-2.54,0,0,0],[0,0,0,-0.75])
                        time.sleep(2)
                        self.send_joint_state_real([1.56,2.94,-2.54,0,0,0],[-1.56,2.94,-2.54,0,0,0],[0,0,0,0])
                        self.send_joint_state_real([1.56,2.94,-2.54,0,0,0],[-1.56,2.94,-2.54,0,0,0],[0,0,0,0])
                    elif command == 'q':
                        self.send_breaking_mode([0]) 
                except KeyboardInterrupt:
                    rospy.loginfo("Keyboard interrupt received, shutting down...")
                    rospy.signal_shutdown("User requested shutdown via Ctrl+C")
                    sys.exit(0)
                    break
        except KeyboardInterrupt:
            print("Exit the program")
        finally:
            # rospy.signal_shutdown()
            print("shutting the ROS node")
            sys.exit(0)     


if __name__ == '__main__':
    try:
        sender = JointStateSender()
        sender.run()
    except rospy.ROSInterruptException:
        pass
