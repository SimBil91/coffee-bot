#!/usr/bin/env python
import rospy
import sys, select, termios, tty
from std_msgs.msg import UInt16
from std_msgs.msg import String
import yaml
import rospkg
import glob, os

class RobotPoses:
	def __init__(self):
		self.current_robot_state = {}
		self.pose_limits = {}
		self.init_current_robot_state()
		self.current_joint = 'x'
		self.current_poses = []
		self.resolution = 10.0 # poses per second
		self.max_acceleration = 100.0

	def init_node(self):
		# ROS related init stuff
		self.pubx = rospy.Publisher('x', UInt16, queue_size=10)
		self.puby = rospy.Publisher('y', UInt16, queue_size=10)
		self.pubz = rospy.Publisher('z', UInt16, queue_size=10)
		self.pubg = rospy.Publisher('g', UInt16, queue_size=10)
		rospack = rospkg.RosPack()
		self.path = rospack.get_path('coffee_bot')
		rospy.Subscriber("pose_name", String, self.callback_fun)

	def callback_fun(self, msg):
		filename = msg.data + '.yaml'
		rospy.loginfo("trying to load " + filename + '...')
		with open(self.path+"/robot_poses/"+filename, 'r') as stream:
		    try:
				robot_poses = yaml.load(stream)
				rospy.loginfo("Playing sequence " + msg.data)
				self.play_pose_sequence(robot_poses)
		    except yaml.YAMLError as exc:
		        print(exc)


	def init_current_robot_state(self):
		self.current_robot_state["x"] = 70 # 0 - 140
		self.current_robot_state["y"] = 180 # 90 - 180
		self.current_robot_state["z"] = 60 # 0 - 180
		self.current_robot_state["g"] = 120 # 80 - 140
		self.current_robot_state["v"] = 10.0

		self.pose_limits["x"] = [0, 140]
		self.pose_limits["y"] = [90, 180]
		self.pose_limits["z"] = [0, 180]
		self.pose_limits["g"] = [80, 140]

	def play_pose_sequence(self, sequence):
		# add current state
		poses = [self.current_robot_state]
		poses[0]["v"] = 0.0
		poses += sequence
		for i in range(0, len(poses) - 1):
			goal_reached = False
			goal_reached_x = False
			goal_reached_y = False
			goal_reached_z = False
			goal_reached_g = False
			rospy.loginfo("Next Pose: ")
			increment_x = float((poses[i+1]["x"] - poses[i]["x"]))
			increment_y = float((poses[i+1]["y"] - poses[i]["y"]))
			increment_z = float((poses[i+1]["z"] - poses[i]["z"]))
			increment_g = float((poses[i+1]["g"] - poses[i]["g"]))
			increment_v = float((poses[i+1]["v"] - poses[i]["v"]))
			if increment_x > 0:
				increment_x = 1
			else:
				increment_x = -1
			if (increment_y > 0):
				increment_y = 1
			else :
				increment_y = -1
			if (increment_z > 0):
				increment_z = 1
			else :
				increment_z = -1
			if (increment_g > 0):
				increment_g = 1
			else :
				increment_g = -1
			if (increment_v > 0):
				increment_v = 1
			else :
				increment_v = -1
			x = poses[i]["x"]
			y = poses[i]["y"]
			z = poses[i]["z"]
			g = poses[i]["g"]
			v = poses[i]["v"]
			print poses
			while not goal_reached:
				v += increment_v * self.max_acceleration
				print v
				if (v > poses[i+1]["v"] and increment_v > 0 or v < poses[i+1]["v"] and increment_v < 0):
					v = poses[i+1]["v"]
				x += increment_x * v / self.resolution
				y += increment_y * v / self.resolution
				z += increment_z * v / self.resolution
				g += increment_g * v / self.resolution
				if (x > poses[i+1]["x"] and increment_x > 0 or x < poses[i+1]["x"] and increment_x < 0):
					x = poses[i+1]["x"]
					goal_reached_x = True
				if (y > poses[i+1]["y"] and increment_y > 0 or y < poses[i+1]["y"] and increment_y < 0):
					y = poses[i+1]["y"]
					goal_reached_y = True
				if (z > poses[i+1]["z"] and increment_z > 0 or z < poses[i+1]["z"] and increment_z < 0):
					z = poses[i+1]["z"]
					goal_reached_z = True
				if (g > poses[i+1]["g"] and increment_g > 0 or g < poses[i+1]["g"] and increment_g < 0):
					g = poses[i+1]["g"]
					goal_reached_g = True
				goal_reached = goal_reached_x and goal_reached_y and goal_reached_z and goal_reached_g
				self.update_current_robot_state(x, y, z, g, v)
				self.publish_current_robot_state(self.current_robot_state)
				rospy.sleep(1.0 / self.resolution)


	def print_current_robot_state(self):
		print self.current_robot_state

	def update_current_robot_state(self, x, y, z, g, v):
		self.current_robot_state["x"] = x
		self.current_robot_state["y"] = y
		self.current_robot_state["z"] = z
		self.current_robot_state["g"] = g
		self.current_robot_state["v"] = v

	def publish_current_robot_state(self, state):
		self.pubx.publish(state["x"])
		self.puby.publish(state["y"])
		self.pubz.publish(state["z"])
		self.pubg.publish(state["g"])

	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key

	def process_key(self):
		# Contains all actions on various key presses
		key = self.getKey()
		if key == 'q':
			rospy.loginfo("Quitting robot poses")
			return False
		elif key == 's':
			rospy.loginfo("Saving current Sequence of Poses")
			print "Choose Filename:"
			filename = raw_input()
			yaml.dump(self.current_poses, open(self.path + "/robot_poses/" + filename+".yaml", "w"), default_flow_style=False)
			rospy.loginfo(self.path + "/" + filename+".yaml saved...")
			self.current_poses = []

		elif key == 'p':
			rospy.loginfo("Test play current Poses")
			self.play_pose_sequence(self.current_poses)
		elif key == 'l':
			rospy.loginfo("Listing all saved poses:")
			os.chdir(self.path + "/robot_poses")
			for file in glob.glob("*.yaml"):
			    print(file)
		elif key == 'n':
			self.current_poses.append(self.current_robot_state.copy())
			print(self.current_poses)
			rospy.loginfo("Record next pose")
		elif key == '1':
			self.current_robot_state["v"] -= 1
			print(self.current_poses)
			rospy.loginfo("New velocity: " + str(self.current_robot_state["v"]))
		elif key == '2':
			self.current_robot_state["v"] += 1
			print(self.current_poses)
			rospy.loginfo("New velocity: " + str(self.current_robot_state["v"]))
		elif key == 'n':
			self.current_poses.append(self.current_robot_state.copy())
			print(self.current_poses)
			rospy.loginfo("Record next pose")
		elif key == 'x':
			rospy.loginfo("Changing x Joint")
			self.current_joint = 'x'
		elif key == 'y':
			rospy.loginfo("Changing y Joint")
			self.current_joint = 'y'
		elif key == 'z':
			rospy.loginfo("Changing z Joint")
			self.current_joint = 'z'
		elif key == 'g':
			rospy.loginfo("Changing g Joint")
			self.current_joint = 'g'
		elif key == 'i':
			rospy.loginfo("Going back to init")
			self.init_current_robot_state()

		elif key == '+':
			if self.current_robot_state[self.current_joint] < self.pose_limits[self.current_joint][1]:
				self.current_robot_state[self.current_joint] += 1
				self.print_current_robot_state()
		elif key == '-':
			if self.current_robot_state[self.current_joint] > self.pose_limits[self.current_joint][0]:
				self.current_robot_state[self.current_joint] -= 1
				self.print_current_robot_state()
		else:
			rospy.loginfo("press: q: Quit, s: Save, p: Play, n: Next, 1: Slower, 2: Faster, Joints: x, y, z, g, + -")
		# PUBLISH CURRENT ROBOT POSE
		self.publish_current_robot_state(self.current_robot_state)

		return True

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('robot_poses', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	robot_pos = RobotPoses()
	robot_pos.init_node()
	while not rospy.is_shutdown():
		if not robot_pos.process_key():
			break
		rate.sleep()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
