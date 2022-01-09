#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped
from math import *
import random


class controller:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('run1', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/drone1/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
	self.local_publisher = rospy.Publisher('/drone1/mavros/setpoint_position/local',PoseStamped, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/drone1/mavros/local_position/pose',PoseStamped, self.update_pose)
 	
        self.pose = PoseStamped()
        self.rate = rospy.Rate(10)
	self.move2goal()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data

    def move2goal(self):
        """Moves the turtle to the goal."""
	count = 1
	vel_msg = TwistStamped()
	vx = 0.2
	vy = 0.2
	vz = 0.2

        while not rospy.is_shutdown():
	    if(count > 60):
		count = 1
		vx = 0.5-random.random()
		vy = 0.5-random.random()
		vz = 0.5-random.random()
	    
	    vel_msg.twist.linear.x = vx
	    vel_msg.twist.linear.y = vy
     	    vel_msg.twist.linear.z = vz
	    vel_msg.twist.angular.z = 0

	    self.velocity_publisher.publish(vel_msg)
	    print "x: "
	    print self.pose.pose.position.x
	    print "y: "
	    print self.pose.pose.position.y
	    print "z: "
	    print self.pose.pose.position.z
	    print "-------------"

            count=count+1
	    print (vx)
	    print (vy)
	    print (vz)
            self.rate.sleep()
	vel_msg.twist.linear.x = 0
        vel_msg.twist.linear.y = 0
	vel_msg.twist.linear.z = 0
        self.velocity_publisher.publish(vel_msg)
	
    def loop(self):
        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = controller()
        x.loop()
    except rospy.ROSInterruptException:
        pass
