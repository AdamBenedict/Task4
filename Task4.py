#!/usr/bin/env python
import rospy
import os
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
PI = 3.1415926535897
i=0
polygon = ""
os.system("killall rosmaster &")
time.sleep(0.3)
os.system("roscore &")
time.sleep(1.75)
os.system("rosrun turtlesim turtlesim_node &")
time.sleep(0.33)

#-----------------------------------------------------------------------------------------
class usual():

	def init(self):
		# Starts a new node
		rospy.init_node('robot_cleaner', anonymous=True)
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=20)
		vel_msg = Twist()
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

		self.pose = Pose()
		self.rate = rospy.Rate(1000)
		# Write Pose Info to File for 15 seconds
		os.system("timeout 30 rostopic echo /turtle1/pose >/home/adam/logs/Task4turtle1Log.txt &")
		#What Shape do you want?
		self.shape = raw_input("What's the Shape?: ")
		global polygon
		polygon = self.shape
		if self.shape in ('square','Square'):
			square().square()
		if self.shape in ('circle','Circle'):
			circle().circle()
		if self.shape in ('triangle','Triangle'):
			triangle().triangle()
		if self.shape in ('rectangle','Rectangle'):
			rect().rect()

		TurtleBot().move2goal()

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = self.pose.x
		self.pose.y = self.pose.y
#-----------------------------------------------------------------------------------------
class math():
	def vel_msg_Calc(self, distance, time):
		return float(distance/(time+0.159))

	def angleCalc(self, NumberOfSides, time):
		return ( (2*PI/NumberOfSides) / ((time+0.159)/NumberOfSides)) #1.5704067955 = 0.033

	def goalCalc(self, currentPos, distance, NumSides):
		return float(currentPos + distance/NumSides)

	def rectgoalCalc(self, currentPos, distance, NumSides):
		return float( (currentPos + distance/NumSides) * float(0.5) )

	def linXCircle(self, radius, time):
		return float((2*PI*radius)/(time+0.159))

	def angularZ(self, linearX, radius):
		return (linearX)/(radius)
#-----------------------------------------------------------------------------------------
class circle():


	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = self.pose.x
		self.pose.y = self.pose.y

	def circle(self):
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=20)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
		self.pose = Pose()
		
		self.radius = raw_input("Radius?: ")
		self.time = raw_input("Time to Finish?: ")

		vel_msg = Twist()
		vel_msg.linear.x = math().linXCircle(float(self.radius),0.5*float(self.time))
		vel_msg.angular.z = math().angularZ( float(vel_msg.linear.x), float(self.radius) )

		while ( float(self.pose.theta) >= float(0) ):
			self.velocity_publisher.publish(vel_msg)

		while ( float(self.pose.theta) < float(-0.015) ):
			self.velocity_publisher.publish(vel_msg)

		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		print 'goodbye'

#-----------------------------------------------------------------------------------------
class square():

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = self.pose.x
		self.pose.y = self.pose.y

	def square(self):
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=20)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
		self.pose = Pose()
		
		self.NumberOfSides = 4
		self.distance = raw_input("Length of Shape?: ")
		self.time = raw_input("Time to Finish?: ")
		
		square.moveRight(self)

	def moveRight(self):
		print "Moving Right"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		goal = math().goalCalc(float(self.pose.x),float(self.distance),float(self.NumberOfSides))

		while ( float(self.pose.x) < float(goal)-0.05 ):
			self.velocity_publisher.publish(vel_msg)
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished Moving Right"
		square.rotate(self)

	def rotate(self):
		vel_msg = Twist()
		vel_msg.angular.z = math().angleCalc( float(self.distance), float(self.time) )
		angle = float(self.pose.theta)
		NumberOfSides = self.NumberOfSides

		if ( self.pose.theta == 0 ): 									#First Rotate
			print "Rotating1"
			while ( ( float(self.pose.theta) < float((2*PI/NumberOfSides))) ):
				self.velocity_publisher.publish(vel_msg)
			square.moveUp(self)


		if ( round(self.pose.theta, 1) == 1.6 ): 						#Second Rotate
			print "Rotating2"
			while ( (self.pose.theta < PI) and (self.pose.theta > 0) ):
				self.velocity_publisher.publish(vel_msg)
			square.moveLeft(self)


		if ( (round(self.pose.theta, 0) == 3) or (round(self.pose.theta, 0) == -3) ): 		#Third Rotate
			print "Rotating3"
			while ( self.pose.theta >= -PI ):
				self.velocity_publisher.publish(vel_msg)
				if (self.pose.theta >= -1.57):
					break
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)
			square.moveDown(self)

	def moveUp(self):
		print "Moving Up"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		distance = float(self.distance)
		NumberOfSides = float(self.NumberOfSides)
		ref = 5.5
		goaly = math().goalCalc(float(self.pose.y),float(self.distance),float(self.NumberOfSides))


		while (float(self.pose.y) < goaly):
			self.velocity_publisher.publish(vel_msg)
			if (self.pose.y > (ref + (distance/NumberOfSides))):
				break

		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print 'Finished Moving Up'

	def moveLeft(self):
		print "Moving Left"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		goal = math().goalCalc(float(self.pose.x),-float(self.distance),float(self.NumberOfSides))

		while ( float(self.pose.x) > float(goal) ):
			self.velocity_publisher.publish(vel_msg)
			if (self.pose.x < goal):
				break
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished Moving Left"



	def moveDown(self):
		print "Moving Down"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		distance = float(self.distance)
		NumberOfSides = float(self.NumberOfSides)
		#goaly = float(self.pose.x)+float(distance)/float(NumberOfSides)
		goaly = math().goalCalc(float(self.pose.y),-float(self.distance),float(self.NumberOfSides))

		# print self.pose.y
		# print goaly
		while (float(self.pose.y) > goaly):
			self.velocity_publisher.publish(vel_msg)

		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print 'Finished Moving Down'

#-----------------------------------------------------------------------------------------
class triangle():

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = self.pose.x
		self.pose.y = self.pose.y

	def triangle(self):
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=20)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
		self.pose = Pose()
		
		self.NumberOfSides = 3
		self.distance = raw_input("Length of Shape?: ")
		self.time = raw_input("Time to Finish?: ")
		triangle.moveRight(self)

	def moveRight(self):
		print "Moving Right"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		goal = math().goalCalc(float(self.pose.x),float(self.distance),float(self.NumberOfSides))

		while ( float(self.pose.x) < float(goal)-0.05 ):
			self.velocity_publisher.publish(vel_msg)
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished Moving Right"
		triangle.rotate(self)

	def rotate(self):
		vel_msg = Twist()
		vel_msg.angular.z = math().angleCalc( float(self.distance), float(self.time) )
		angle = float(self.pose.theta)
		NumberOfSides = self.NumberOfSides

		if ( self.pose.theta == 0 ): 									#First Rotate
			print "Rotating1"
			while ( ( float(self.pose.theta) < float((2*PI/NumberOfSides))) ):
				self.velocity_publisher.publish(vel_msg)
			triangle.moveUp(self)


		if ( round(self.pose.theta, 1) == round( (2*PI/NumberOfSides) ,1) ): 						#Second Rotate
			print "Rotating2"
			while True:
				self.velocity_publisher.publish(vel_msg)
				if ( round(self.pose.theta, 1) == round( (-2*PI/3),1) ):
					break
			triangle.moveLeft(self)
		
		if ( round(self.pose.theta,1) < 0 ):
			while True:
				self.velocity_publisher.publish(vel_msg)
				if ( round(self.pose.theta, 2) > -0.025 ):
					break
			triangle.moveRight(self)

	def moveUp(self):
		print "Moving Up"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		distance = float(self.distance)
		NumberOfSides = float(self.NumberOfSides)
		ref = 5.5
		goaly = math().goalCalc(float(self.pose.y),float(self.distance),float(self.NumberOfSides))


		while (float(self.pose.y) < goaly):
			self.velocity_publisher.publish(vel_msg)
			if (self.pose.y > (ref + (distance/NumberOfSides))):
				break

		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print 'Finished Moving Up'

	def moveLeft(self):
		print "Moving Left"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		goal = math().goalCalc(float(self.pose.x),-float(self.distance),float(self.NumberOfSides))

		while ( float(self.pose.x) > float(goal) ):
			self.velocity_publisher.publish(vel_msg)
			if (round(self.pose.y,1) == 5.5):
				break
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished Moving Left"
		if ( (self.pose.x) < 5.544445):
			triangle.rotate(self)
#-----------------------------------------------------------------------------------------
class rect():

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = self.pose.x
		self.pose.y = self.pose.y

	def rect(self):
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=20)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
		self.pose = Pose()
		
		self.NumberOfSides = 4
		self.distance = raw_input("Length of Shape?: ")
		self.time = raw_input("Time to Finish?: ")
		
		rect.moveRight(self)

	def moveRight(self):
		print "Moving Right"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		goal = math().goalCalc(float(self.pose.x),float(self.distance),float(self.NumberOfSides))

		while ( float(self.pose.x) < float(goal)-0.05 ):
			self.velocity_publisher.publish(vel_msg)
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished Moving Right"
		rect.rotate(self)

	def rotate(self):
		vel_msg = Twist()
		vel_msg.angular.z = math().angleCalc( float(self.distance), float(self.time) )
		angle = float(self.pose.theta)
		NumberOfSides = self.NumberOfSides

		if ( self.pose.theta == 0 ): 									#First Rotate
			vel_msg.angular.z = math().angleCalc( float(self.distance), float(self.time) )
			print "Rotating1"
			while ( self.pose.theta < 1.57 ):
				self.velocity_publisher.publish(vel_msg)
				if (self.pose.theta >= 1.57):
					break
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)
			rect.moveUp(self)


		if ( round(self.pose.theta, 1) == 1.6 ): 						#Second Rotate
			vel_msg.angular.z = math().angleCalc( float(self.distance), float(self.time) )
			print "Rotating2"
			while ( self.pose.theta > 0 ):
				self.velocity_publisher.publish(vel_msg)
				if (self.pose.theta < 0):
					break
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)
			rect.moveLeft(self)


		if ( (round(self.pose.theta, 0) == 3) or (round(self.pose.theta, 0) == -3) ): 		#Third Rotate
			vel_msg.angular.z = math().angleCalc( float(self.distance), float(self.time) )
			print "Rotating3"
			while ( self.pose.theta >= -PI ):
				self.velocity_publisher.publish(vel_msg)
				if (self.pose.theta >= -1.57):
					break
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)
			rect.moveDown(self)

	def moveUp(self):
		print "Moving Up"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		distance = float(self.distance)
		NumberOfSides = float(self.NumberOfSides)
		goaly = math().goalCalc(float(self.pose.y),float(self.distance),float(self.NumberOfSides))

		halfGoalY = float(self.pose.y + (0.125*goaly) )

		while ( float(self.pose.y) < halfGoalY):
			self.velocity_publisher.publish(vel_msg)

		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print 'Finished Moving Up'

	def moveLeft(self):
		print "Moving Left"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))

		goal = math().goalCalc(float(self.pose.x),-float(self.distance),float(self.NumberOfSides))

		print "Pose.x is {}".format(self.pose.x)
		print "Goal is {}".format(goal)
		while ( float(self.pose.x) > float(goal) ):
			self.velocity_publisher.publish(vel_msg)
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished Moving Left"



	def moveDown(self):
		print "Moving Down"
		vel_msg = Twist()
		vel_msg.linear.x = math().vel_msg_Calc(float(self.distance),float(self.time))
		
		while (float(self.pose.y) > 5.5):
			self.velocity_publisher.publish(vel_msg)

		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		print 'Finished Moving Down'
#-----------------------------------------------------------------------------------------
class TurtleBot:

	def __init__(self):

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle2/cmd_vel',
				                                  Twist, queue_size=10)

		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('/turtle2/pose',
				                                Pose, self.update_pose)

		self.pose = Pose()
		self.rate = rospy.Rate(10)

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.x - self.pose.x), 2) +
				    pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		if polygon in ('square','Square', 'rectangle', 'Rectangle', 'triangle', 'Triangle'):
			return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

		if polygon in ('circle','Circle'):
			return abs(atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x))

	def angular_vel(self, goal_pose, constant=6):
		if polygon in ('square','Square', 'rectangle', 'Rectangle'):
			if (self.steering_angle(goal_pose) < 0):
				return constant * (self.steering_angle(goal_pose) + abs(self.pose.theta) )
			else:
				return constant * (self.steering_angle(goal_pose) - self.pose.theta)

		if polygon in ('circle','Circle'):
			return abs(constant * (self.steering_angle(goal_pose) - abs(self.pose.theta)) )

		if polygon in ('triangle','Triangle'):
			return constant * (self.steering_angle(goal_pose) - self.pose.theta)

	def move2goal(self):

		x1 = []
		y1 = []


		with open("/home/adam/logs/Task4turtle1Log.txt") as turtleData:
			for line in turtleData:
				data = line.strip()
				if line.startswith('x'):
					x1.append(float(data[3:]))
				if line.startswith('y'):
					y1.append(float(data[3:]))
		os.system("rosservice call /spawn 5.544445 5.544445 0.0 turtle2")
		goal_pose = Pose()

		print "Moving Turtle2"
		for i in range( 0, len(x1) ):
			goal_pose.x = x1[i]
			goal_pose.y = y1[i]
			

			distance_tolerance = 0.1
			vel_msg = Twist()

			while self.euclidean_distance(goal_pose) >= distance_tolerance:

				# Porportional controller.
				# https://en.wikipedia.org/wiki/Proportional_control

				# Linear velocity in the x-axis.
				vel_msg.linear.x = self.linear_vel(goal_pose)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel(goal_pose)

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

				# Publish at the desired rate.
				self.rate.sleep()

			i =i+1
		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		print "Finished"
		sys.exit()

		# If we press control + C, the node will stop.
		rospy.spin()
		
#-----------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:
		usual().init()

	except:
		pass

		#hacker rank
        #leet code
