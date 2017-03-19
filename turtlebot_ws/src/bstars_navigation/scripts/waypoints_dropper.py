#!/usr/bin/env python2
"""
Node to allow quick waypoint dropping to file
"""

from __future__ import print_function
import rospy
import roslib
import rospkg
import os.path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import tf
from PyQt4 import QtGui, QtCore
import sys



class WaypointDropper(QtGui.QWidget):
    def __init__(self):
        super(WaypointDropper, self).__init__()
        self.initUI()

        # get the filename to be saved to
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('bstars_navigation') + '/param/'

        name_chosen = False
        while(not name_chosen and not rospy.is_shutdown()):
            filename = raw_input('Enter name of file to save: ') + '.txt'
            if os.path.isfile(save_path + filename):
                choice = raw_input("File " + filename + " exists. Append to file? ")
                if choice == "y" or choice == "yes":
                    name_chosen = True
            else:
                name_chosen = True

        self.save_file = save_path + filename

        # tf listener
        self.listener = tf.TransformListener()
        self.current_pose = [0, 0, 0]

    def initUI(self):
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Waypoint Dropper')
        self.show()

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Space:
            self.get_tf()
        elif e.key() == QtCore.Qt.Key_Escape:
            self.close()

    def get_tf(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rpy = euler_from_quaternion(rot)
            yaw = rpy[2]
            self.current_pose = [trans[0], trans[1], yaw]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf not available yet")

        self.save_point()

    def save_point(self):
        print("%f, %f, %f recorded." % (self.current_pose[0], self.current_pose[1], self.current_pose[2]))
        with open(self.save_file, 'a') as f:
            f.write(str(self.current_pose[0]) + ' ' +
                    str(self.current_pose[1]) + ' ' +
                    str(self.current_pose[2]) + '\n')


if __name__ == '__main__':
    rospy.init_node('waypoint_dropper')
    try:
        gui = QtGui.QApplication(sys.argv)
        wd = WaypointDropper()
        sys.exit(gui.exec_())
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass