#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import Joy

# Declare max velocities 
MAXV_TX = 0.05
MAXV_TY = 0.05
MAXV_TZ = 0.08
MAXV_RX = 0.5
MAXV_RY = 0.3
MAXV_RZ = 0.5
MAXV_GR = 0.3 # gripper

RETRACT_ACTION_IDENTIFIER = 1
HOME_ACTION_IDENTIFIER = 2

class IrisControl:
	def __init__(self):
		# global 
		self.run = True

		self.last_action_notif_type = None
		self.zero_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.axes_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.prev_cv_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.gripper_cmd = 0.0
		self.prev_gripper_cmd = 0.0

		# Get node params
		self.robot_name = rospy.get_param('~robot_name', "iris")
		self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
		self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)
		self.input_device = rospy.get_param('~input_device', "flightstick")
		self.control_scheme = rospy.get_param('~control_scheme', "6D")
		self.reference_frame = rospy.get_param('~reference_frame', "mixed")
		# rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + \
			# " degrees of freedom, is_gripper_present is " + str(self.is_gripper_present) + \
			# ", using input device " + self.input_device + " and control scheme " + self.control_scheme)
		
		# Init subscribers and publishers
		self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.action_topic_callback)
		self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
		self.cartesian_velocity_pub = rospy.Publisher("/" + self.robot_name + "/in/cartesian_velocity", TwistCommand, queue_size=10)

		# Init services
		clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
		rospy.wait_for_service(clear_faults_full_name)
		self.clear_faults_srv = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

		read_action_full_name = '/' + self.robot_name + '/base/read_action'
		rospy.wait_for_service(read_action_full_name)
		self.read_action_srv = rospy.ServiceProxy(read_action_full_name, ReadAction)

		execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
		rospy.wait_for_service(execute_action_full_name)
		self.execute_action_srv = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

		# stop_motion = '/' + self.robot_name + '/base/stop'
		# rospy.wait_for_service(execute_action_full_name)
		# self.execute_action_srv = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

		set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
		rospy.wait_for_service(set_cartesian_reference_frame_full_name)
		self.set_cartesian_reference_frame_srv = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

		play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
		rospy.wait_for_service(play_cartesian_trajectory_full_name)
		self.play_cartesian_trajectory_srv = rospy.ServiceProxy(play_cartesian_trajectory_full_name, PlayCartesianTrajectory)

		play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
		rospy.wait_for_service(play_joint_trajectory_full_name)
		self.play_joint_trajectory_srv = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

		send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
		rospy.wait_for_service(send_gripper_command_full_name)
		self.gripper_command_srv = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

		activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
		rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
		self.activate_publishing_of_action_notification_srv = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

		get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
		rospy.wait_for_service(get_product_configuration_full_name)
		self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)
		
		self.print_menu()

	def shutdown(self):
		# success = self.send_action_command()
		# if not success:
			# rospy.logerr("Error moving back to home position on shutdown.")
		rospy.loginfo("Shutting down control node.")

	def action_topic_callback(self, notif):
		self.last_action_notif_type = notif.action_event

	def joy_callback(self, msg):
		# rospy.loginfo(rospy.get_caller_id() + " axes data: %s", str(msg.axes))
		self.axes_vector = msg.axes
		
		# check for gripper commands
		if msg.buttons[0]: # trigger button - close gripper
			self.gripper_cmd = -1 * MAXV_GR
		elif msg.buttons[1]: # button by thumb - open gripper
			self.gripper_cmd = MAXV_GR
		else: # both buttons 0 and 1 are zero
			self.gripper_cmd = 0.0

		# button 6 pressed, shutdown node
		if msg.buttons[6]: 
			self.run = False
			rospy.signal_shutdown("Button 6 pressed: initiating shutdown sequence.") 
			rospy.sleep(0.1)
		
		# button 8 pressed, send robot home
		if msg.buttons[8]:
			rospy.loginfo("Button 8 pressed: sending robot home.")
			success = self.send_action_command(HOME_ACTION_IDENTIFIER)
			if not success:
				rospy.logerr("Error moving back to home position.")

		# button 9 pressed, retract robot
		if msg.buttons[9]:
			rospy.loginfo("Button 9 pressed: sending robot to retract position.")
			success = self.send_action_command(RETRACT_ACTION_IDENTIFIER)
			if not success:
				rospy.logerr("Error moving back to retract position.")

		# button 10 pressed, stop robot motion
		if msg.buttons[10]:
			rospy.loginfo("Button 10 pressed: stopping motion")
			self.send_cartesian_velocity(1, self.zero_vector)

	def wait_for_action_end_or_abort(self):
		while not rospy.is_shutdown():
			if (self.last_action_notif_type == ActionEvent.ACTION_END):
				rospy.loginfo("Received ACTION_END notification")
				return True
			elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
				rospy.loginfo("Received ACTION_ABORT notification")
				return False
			else:
				time.sleep(0.01)

	def subscribe_to_a_robot_notification(self):
		# Activate the publishing of the ActionNotification
		req = OnNotificationActionTopicRequest()
		rospy.loginfo("Activating the action notifications...")
		try:
			self.activate_publishing_of_action_notification_srv(req)
		except rospy.ServiceException:
			rospy.logerr("Failed to call OnNotificationActionTopic")
			return False
		else:
			rospy.loginfo("Successfully activated the Action Notifications!")
		rospy.sleep(1.0)
		return True

	def clear_faults(self):
		try:
			self.clear_faults_srv()
		except rospy.ServiceException:
			rospy.logerr("Failed to call ClearFaults")
			return False
		else:
			rospy.loginfo("Cleared the faults successfully")
			rospy.sleep(2.5)
			return True

	def send_action_command(self, action_identifier):
		# The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
		self.last_action_notif_type = None
		req = ReadActionRequest()
		# req.input.identifier = self.HOME_ACTION_IDENTIFIER
		req.input.identifier = action_identifier
		try:
			res = self.read_action_srv(req)
		except rospy.ServiceException:
			rospy.logerr("Failed to call ReadAction")
			return False
		# Execute the HOME action if we could read it
		else:
			# What we just read is the input of the ExecuteAction service
			req = ExecuteActionRequest()
			req.input = res.output
			rospy.loginfo("Sending the robot home...")
			try:
				self.execute_action_srv(req)
			except rospy.ServiceException:
				rospy.logerr("Failed to call ExecuteAction")
				return False
			else:
				return self.wait_for_action_end_or_abort()

	def set_cartesian_reference_frame(self):
		# CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0
		# CARTESIAN_REFERENCE_FRAME_MIXED = 1
		# CARTESIAN_REFERENCE_FRAME_TOOL = 2
		# CARTESIAN_REFERENCE_FRAME_BASE = 3
		self.last_action_notif_type = None
		# Prepare the request with the frame we want to set
		req = SetCartesianReferenceFrameRequest()
		req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
		try:
			self.set_cartesian_reference_frame_srv()
		except rospy.ServiceException:
			rospy.logerr("Failed to call SetCartesianReferenceFrame")
			return False
		else:
			rospy.loginfo("Set the cartesian reference frame successfully")
		rospy.sleep(0.25)
		return True

	def send_cartesian_pose(self):
		self.last_action_notif_type = None
		# Get the actual cartesian pose to increment it
		# You can create a subscriber to listen to the base_feedback
		# Here we only need the latest message in the topic though
		feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
		req = PlayCartesianTrajectoryRequest()
		req.input.target_pose.x = feedback.base.commanded_tool_pose_x
		req.input.target_pose.y = feedback.base.commanded_tool_pose_y
		req.input.target_pose.z = feedback.base.commanded_tool_pose_z + 0.10
		req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
		req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
		req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

		pose_speed = CartesianSpeed()
		pose_speed.translation = 0.1
		pose_speed.orientation = 15
		# The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
		# To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object : 
		req.input.constraint.oneof_type.speed.append(pose_speed)
		# Call the service
		rospy.loginfo("Sending the robot to the cartesian pose...")
		try:
			self.play_cartesian_trajectory_srv(req)
		except rospy.ServiceException:
			rospy.logerr("Failed to call PlayCartesianTrajectory")
			return False
		else:
			return self.wait_for_action_end_or_abort()

	def send_cartesian_velocity(self, ref_frame, axes):
		# CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0,
		# CARTESIAN_REFERENCE_FRAME_MIXED = 1,
		# CARTESIAN_REFERENCE_FRAME_TOOL = 2,
		# CARTESIAN_REFERENCE_FRAME_BASE = 3,
		cmd = TwistCommand()
		cmd.reference_frame = ref_frame
		cmd.duration = 0
		cmd.twist.linear_x = axes[0] * MAXV_TX
		cmd.twist.linear_y = axes[1] * MAXV_TY
		cmd.twist.linear_z = axes[2] * MAXV_TZ
		cmd.twist.angular_x = axes[3] * MAXV_RX
		cmd.twist.angular_y = axes[4] * MAXV_RY
		cmd.twist.angular_z = axes[5] * MAXV_RZ
		rospy.loginfo("Sending the cartesian velocity command: [%s, %s, %s, %s, %s, %s]", \
			str(cmd.twist.linear_x), str(cmd.twist.linear_y), str(cmd.twist.linear_z), str(cmd.twist.angular_x), str(cmd.twist.angular_y), str(cmd.twist.angular_z))
		try:
			self.cartesian_velocity_pub.publish(cmd)
		except rospy.ServiceException:
			rospy.logerr("Failed to publish send_cartesian_velocity")
			return False
		else:
			# time.sleep(0.5)
			return True

	def send_joint_angles(self):
		self.last_action_notif_type = None
		# Create the list of angles
		req = PlayJointTrajectoryRequest()
		# Here the arm is vertical (all zeros)
		for i in range(self.degrees_of_freedom):
			temp_angle = JointAngle() 
			temp_angle.joint_identifier = i
			temp_angle.value = 0.0
			req.input.joint_angles.joint_angles.append(temp_angle)
		# Send the angles
		rospy.loginfo("Sending the robot vertical...")
		try:
			self.play_joint_trajectory_srv(req)
		except rospy.ServiceException:
			rospy.logerr("Failed to call PlayJointTrajectory")
			return False
		else:
			return self.wait_for_action_end_or_abort()

	def send_gripper_command(self, value):
		# Initialize the request
		if self.is_gripper_present:
			req = SendGripperCommandRequest()
			finger = Finger()
			finger.finger_identifier = 0
			finger.value = value
			req.input.gripper.finger.append(finger)
			# req.input.mode = GripperMode.GRIPPER_POSITION
			req.input.mode = GripperMode.GRIPPER_SPEED
			# rospy.loginfo("Sending the gripper command: " +  str(value))
			try:
				self.gripper_command_srv(req)
			except rospy.ServiceException:
				rospy.logerr("Failed to call SendGripperCommand")
				return False
			else:
				# time.sleep(0.01)
				return True
		else:
			rospy.logwarn("No gripper is present on the arm.")

	def print_menu(self):
		if self.input_device == "keyboard":
			print("Keyboard")
		elif self.input_device == "joystick_box":
			print("Joystick Box")
		elif self.input_device == "flightstick":
			print("Flightstick")
		elif self.input_device == "spacemouse":
			print("SpaceMouse")
		else:
			print("Unknown Input Device")
		rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + \
			" degrees of freedom, is_gripper_present is " + str(self.is_gripper_present) + \
			", using input device " + self.input_device + ", control scheme " + self.control_scheme + " and ref frame " + self.reference_frame)

	def remap_axes(self, vec):
		ref_frame = 1
		cmd = self.zero_vector

		# 6 DOF robot control using the flightstick with translation in the base frame and orientation in tool frame
		if self.control_scheme == "6D" and self.input_device == "flightstick":
			ref_frame = 1
			cmd = [vec[5], vec[4], vec[3], vec[1], -1.0 * vec[0], -1.0 * vec[2]]
			# deadzone since doesnt reset to zero automatically, easy to accidentally bump
			if abs(cmd[2]) < 0.5: 
				cmd[2] = 0.0

		elif self.control_scheme == "navray" and self.input_device == "flightstick": # pitch, yaw, and forward/backward
			ref_frame = 2
			# cmd = [0.0, 0.0, vec[5], vec[1], -1.0 * vec[0], 0.0]
			cmd = [0.0, 0.0, vec[5], vec[1], vec[2], 0.0]

		if cmd != self.prev_cv_cmd: 
			self.send_cartesian_velocity(ref_frame, cmd)
			self.prev_cv_cmd = cmd

	def step(self):
		# print("Step: ", str(self.axes_vector))
		if self.run:
			self.remap_axes(self.axes_vector)
			# print(str(self.gripper_cmd) + " : " + str(self.prev_gripper_cmd))
			if self.gripper_cmd != self.prev_gripper_cmd:
				success = self.send_gripper_command(self.gripper_cmd)
				self.prev_gripper_cmd = self.gripper_cmd

	# def wait_for_action_end_or_abort(self):
	#   # while not rospy.is_shutdown():
	#   while True:
	#       if (self.last_action_notif_type == ActionEvent.ACTION_END):
	#           rospy.loginfo("Received ACTION_END notification")
	#           return True
	#       elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
	#           rospy.loginfo("Received ACTION_ABORT notification")
	#           return False
	#       else:
	#           time.sleep(0.01)

	# def send_action_command(self):
	#   # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
	#   self.last_action_notif_type = None
	#   # self.clear_faults
	#   req = ReadActionRequest()
	#   req.input.identifier = self.HOME_ACTION_IDENTIFIER
	#   try:
	#       res = self.read_action_srv(req)
	#   except rospy.ServiceException:
	#       rospy.logerr("Failed to call ReadAction")
	#       return False
	#   # Execute the HOME action if we could read it
	#   else:
	#       # What we just read is the input of the ExecuteAction service
	#       req = ExecuteActionRequest()
	#       req.input = res.output
	#       rospy.loginfo("Sending the robot home...")
	#       try:
	#           self.execute_action_srv(req)
	#       except rospy.ServiceException:
	#           rospy.logerr("Failed to call ExecuteAction")
	#           return False
	#       else:
	#           # rospy.sleep(5.0)
	#           # rospy.loginfo("Robot is in home position.")
	#           # return True
	#           return self.wait_for_action_end_or_abort()


if __name__ == "__main__":
	try:
		rospy.init_node('iris_control', anonymous=True)
		robot = IrisControl()
		# rospy.on_shutdown(robot.shutdown)
		# if not success:
			# rospy.logerr("The robot encountered an error while going home.")
		success = robot.clear_faults()
		success &= robot.subscribe_to_a_robot_notification()
		success &= robot.send_action_command(HOME_ACTION_IDENTIFIER)
		if not success:
			rospy.logerr("The robot encountered an error while initializing.")
		rate = rospy.Rate(40) # 40hz
		while not rospy.is_shutdown():
			robot.step()
			rate.sleep()

	except rospy.ROSInterruptException:
		print("ROSInterruptException")

