#! /usr/bin/env python2

from __future__ import print_function

import roslib
import rospy
import actionlib
import tf
#import actionlib.action_client

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatus

class StateMachine():
    def __init__(self):
        self._state = 0
        self._final_waypoint = MoveBaseGoal()
        self._final_waypoint.target_pose.header.frame_id = "map"
        self._ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # initial pose
        self._initial_pose_set = False
        self._initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self._initial_pose_callback)

        # publishers
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/switch', Twist, queue_size=10)

        # tf listener for current pose
        self.listener = tf.TransformListener()

        while not self._ac.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        #self._main_loop_timer = rospy.Timer(rospy.Duration(0.1), self._main_loop)

        self._state_start_time = rospy.get_time()
        if self._state == 0:
            print("waiting for user to set initial pose...")
        self.rate = rospy.Rate(10)
        self._main_loop()

    def _initial_pose_callback(self, msg):
        self._initial_pose_set = True

    def get_pose_as_goal(self):
        # returns trans in [0], rot in [1] as quaternion
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            out = MoveBaseGoal()
            out.target_pose.pose.position.x = trans[0]
            out.target_pose.pose.position.y = trans[1]
            out.target_pose.pose.position.z = trans[2]
            out.target_pose.pose.orientation.x = rot[0]
            out.target_pose.pose.orientation.y = rot[1]
            out.target_pose.pose.orientation.z = rot[2]
            out.target_pose.pose.orientation.w = rot[3]
            return out
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf not available yet")
            return 0

    def _main_loop(self):
        while not rospy.is_shutdown():

            if self._state == 0:
                # wait for initial pose to be set. may need to switch this to waiting for a key press
                if self._initial_pose_set:
                    print("Initial pose set. Moving backwards out of base...")
                    self._state = 1
                    self._state_start_time = rospy.get_time()

            elif self._state == 1:
                # move the robot backward at constant velocity for some time
                if rospy.get_time() - self._state_start_time < 5.0:
                    vel = -0.2
                    cmd_vel = Twist()
                    cmd_vel.linear.x = vel
                    self._cmd_vel_pub.publish(cmd_vel)
                else:
                    print("Robot (probably) out of base. Starting spinning to improve localization convergence.")
                    self._state = 2
                    cur_pose_as_goal = self.get_pose_as_goal()
                    self._final_waypoint.target_pose.pose.orientation = cur_pose_as_goal.target_pose.pose.orientation
                    self._state_start_time = rospy.get_time()

            elif self._state == 2:
                # spin robot to localize, then set point as final waypoint
                if rospy.get_time() - self._state_start_time < 6.28:
                    vel = 2
                    cmd_vel = Twist()
                    cmd_vel.angular.z = vel
                    self._cmd_vel_pub.publish(cmd_vel)
                else:
                    print("Spinning complete. Storing current pose as final waypoint, then starting waypoints.")
                    self._state = 3
                    cur_pose_as_goal = self.get_pose_as_goal()
                    self._final_waypoint.target_pose.pose.position = cur_pose_as_goal.target_pose.pose.position
                    print("test ", self._final_waypoint.target_pose.pose.orientation)
                    self._state_start_time = rospy.get_time()

            elif self._state == 3:
                # initialize the waypoints node, go to all of the waypoints. wait for that node to finish
                print("todo")
                self._state = 4
                self._state_start_time = rospy.get_time()
                #todo: probably going to have to make waypoints_node an acition server OR do publishing/subscribing to communicate

            elif self._state == 4:
                # possible state for checking preliminary sentence quality and doing a 2nd lap. may not use.
                self._ac.send_goal(self._final_waypoint)
                self._state = 5
            elif self._state == 5:
                # go to the final waypoint, prepare to enter dock

                if self._ac.get_state() == GoalStatus.SUCCEEDED:
                    print("Final waypoint reached. Attempting to enter dock.")
                    self._state_start_time = rospy.get_time()
            elif self._state == 6:
                print("todo")
                # attempt to enter dock. if timeout OR sucessfully docked, go to next state
            elif self._state == 7:
                print("todo")
                # sentence analysis/speaking

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('main_state_machine')

    try:
        sm = StateMachine()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

    '''
    client = actionlib.SimpleActionClient('do_dishes', DoDishesAction)
    client.wait_for_server()

    goal = DoDishesGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    '''