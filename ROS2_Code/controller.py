#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name control which holds the position of  Drone .
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error				/throttle_pid
								/pitch_pid
								/roll_pid
					

'''

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node




class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.07, 0.08, 32.24]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2.07, 2.08, 51.24]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_error = [0.0, 0.0, 0.0]
		self.min_values = [1000, 1000, 1000]
		self.max_values = [2000, 2000, 2000]
		self.error = [0.0, 0.0, 0.0]
		self.integral = [0.0 ,0.0, 0.0]
		self.derivative = [0.0, 0.0, 0.0]
		
		

		


		#----------------------------------------------------------------------------------------------------------

	
		self.sample_time = 0.028  # in seconds

		# Publishing /drone_command, /throttle_error, /pitch_error, /roll_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
		self.timer = self.create_timer(self.sample_time, self.pid)	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid , 1)
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)


		#------------------------Add other ROS Subscribers here-----------------------------------------------------
	

		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)


	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, msg):
		
		self.drone_position[0] = msg.poses[0].position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		
	
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 1 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 1
		self.Kd[2] = alt.kd * 1
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll-------------
	
 
    # Callback function for /roll_pid
	# This function gets executed each time when /drone_pid_tuner publishes /roll_pid
	
	def roll_set_pid(self, roll): 
		self.Kp[0] = roll.kp * 1 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.ki * 1
		self.Kd[0] = roll.kd * 1
	#----------------------------------------------------------------------------------------------------------------------
	# Callback function for /pitch_pid
	# This function gets executed each time when /drone_pid_tuner publishes /pitch_pid
	def pitch_set_pid(self, pit): 
		self.Kp[1] = pit.kp * 1 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pit.ki * 1
		self.Kd[1] = pit.kd * 1


	def pid(self):
	#----------------------------- PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
        
		self.error[0] = self.setpoint[0] - self.drone_position[0] #calculating error for x axis
		print(self.error[0])
		self.error[1] = self.setpoint[1] - self.drone_position[1] #calculating error for y axis
		self.error[2] = self.setpoint[2] - self.drone_position[2] #calculating error for z axis
		self.integral[0] += self.error[0] #calculating error over time for x axis
		self.integral[1] += self.error[1] #calculating error over time for y axis
		self.integral[2] += self.error[2] #calculating error over time for z axis
		self.derivative[0] = (self.error[0] - self.prev_error[0]) #calculating change in error for x axis
		self.derivative[1] = (self.error[1] - self.prev_error[1]) #calculating change in error for y axis
		self.derivative[2] = (self.error[2] - self.prev_error[2]) #calculating change in error for z axis
		#print(self.derivative[0])
		self.out_roll = self.Kp[0]*self.error[0] + self.Kd[0]*self.derivative[0] + self.Ki[0]*self.integral[0] #calcuating pid required for x axis
		self.out_pitch = self.Kp[1]*self.error[1] + self.Kd[1]*self.derivative[1] + self.Ki[1]*self.integral[1] #calculating pid for y axis
		self.out_throttle = self.Kp[2]*self.error[2] + self.Kd[2]*self.derivative[2] + self.Ki[2]*self.integral[2] #calculating pid for z axis
		self.cmd.rc_roll = 1500 + int(self.out_roll)
		self.cmd.rc_pitch = 1500 + int(self.out_pitch)
		self.cmd.rc_throttle = 1500 + int(self.out_throttle)
		
		if self.cmd.rc_pitch > self.max_values[1]:
			self.cmd.rc_pitch = self.max_values[1]
		if self.cmd.rc_roll > self.max_values[0]:
			self.cmd.rc_roll = self.max_values[0]
		if self.cmd.rc_throttle > self.max_values[2]:
			self.cmd.rc_throttle = self.max_values[2]
		if self.cmd.rc_pitch < self.min_values[1]:
			self.cmd.rc_pitch = self.min_values[1]
		if self.cmd.rc_roll < self.min_values[0]:
			self.cmd.rc_roll = self.min_values[0]
		if self.cmd.rc_throttle < self.min_values[2]:
			self.cmd.rc_throttle = self.min_values[2]
		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1]
		self.prev_error[2] = self.error[2]
		'''self.get_logger().info('PID is working')
		self.get_logger().info(f'p_error={self.error}')
		self.get_logger().info(f'd_error={self.derivative}')
		self.get_logger().info(f'i_error={self.integral}')
		self.get_logger().info(f'roll_pid={self.cmd.rc_roll}')
		self.get_logger().info(f'pitch_pid={self.cmd.rc_pitch}')
		self.get_logger().info(f'throttle_pid={self.cmd.rc_throttle}')'''










	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		# calculate throttle error, pitch error and roll error, then publish it accordingly
		self.pid_error = PIDError()
		self.pid_error._throttle_error = self.setpoint[2] - self.drone_position[2]
		self.pid_error._pitch_error = self.setpoint[1] - self.drone_position[1]
		self.pid_error._roll_error = self.setpoint[0] - self.drone_position[0]
		self.pid_error_pub.publish(self.pid_error)

  		



def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	
	
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	
	rclpy.shutdown()


if __name__ == '__main__':
	main()
