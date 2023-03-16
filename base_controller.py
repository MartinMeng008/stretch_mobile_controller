#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cs4750 import utils
import time

DEBUG = False

class BaseController:

    def __init__(self, x: np.float = None, y: np.float = None, t: np.float = None):
        # Setup
        self.path = np.array([[5, 0, 5]])  # pose = [x, y, theta]
        self.progress_threshold = 0.2
        self.progress_time_threshold = 20.0
        self.prev_pose = self.curr_pose = None

        self.kv = 1
        self.kw = 0.7
        if t is None:
            self.set_path([[x, y]])
        else:
            self.set_path([[x, y, t]])
        self.finish_threshold = 0.1
        self.finish_threshold_theta = 0.1
        self.frequency = 50

        # Create publisher and subscriber
        rospy.init_node('mobile_control', anonymous=True)
        self.rate = rospy.Rate(self.frequency)
        self.pub = rospy.Publisher(
            '/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/ground_truth', Odometry, self.callback)
        # rospy.spin()

    #### Control loop ####
    def control_loop(self):
        rospy.logdebug(rospy.get_caller_id() + " Start control loop")
        self.wrap_path_radians(self.path)
        gotopt = 0
        while not rospy.is_shutdown():
            if self.curr_pose is None:
                continue
            if self._no_progress():
                rospy.logerr(rospy.get_caller_id(
                ) + " Error: No progress after %ss -- Current pose: %s" % (self.progress_time_threshold, np.array2string(self.curr_pose)))
                break
            gotopt = self.get_reference_index(self.curr_pose, self.path, gotopt)
            if self._path_complete(self.path, gotopt):
                rospy.loginfo(rospy.get_caller_id() + " Reach the end of path.")

                rospy.loginfo(rospy.get_caller_id() + " -- Path completed -- Current pose: %s -- Goal pose: %s" %
                              (np.array2string(self.curr_pose), np.array2string(self.goal_pose)))
                break
            self.goal_pose = np.array(self.path[gotopt])
            self.error = self._get_error(self.curr_pose, self.goal_pose)
            self.next_control = self._get_control(
                self.curr_pose, self.goal_pose, self.error)
            if DEBUG:
                rospy.logdebug(rospy.get_caller_id() + "Control: %s" %
                           np.array2string(self.next_control))
            self._publish_control(self.next_control)
            self.rate.sleep()

    def control_loop_theta(self, target: np.ndarray, gotopt: int) -> None:
        """After reaching x, y, rotate theta to the target location"""
        rospy.loginfo(rospy.get_caller_id() + " Start theta control loop")
        while not rospy.is_shutdown():
            pose = self.curr_pose
            if self._no_progress():
                rospy.logerr(rospy.get_caller_id(
                ) + " Error: No progress after 20s -- Current pose: %s" % np.array2string(pose))
                break
            self.error = self._get_error_theta(pose, target)
            if self._path_complete_theta(self.error):
                rospy.loginfo(rospy.get_caller_id() + " -- Rotation completed for waypoint %s -- Current pose: %s -- Goal pose: %s" %
                              (gotopt, np.array2string(pose), np.array2string(target)))
                break
            self.next_control = self._get_control_theta(self.error)
            if DEBUG:
                rospy.loginfo(rospy.get_caller_id() + " Control: %s" %
                          np.array2string(self.next_control))
            self._publish_control(self.next_control)
            self.rate.sleep()

    def _get_control(self, pose: np.ndarray, target: np.ndarray, error: np.ndarray) -> np.ndarray:
        """Compute the PD control

        Args:
            pose: np.ndarray: current pose of the car [x, y, heading]
            target: np.ndarray: target pose [x, y, theta, v]
            error: np.ndarray error [e_x, e_y, e_heading]

        Returns:
            control: np.ndarray: linear velocity and steering angle [v_x, w_z]
        """
        raise NotImplementedError

    #### Helper functions ####

    def _get_error(self, pose: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Get the absolute difference between the current pose and the target pose"""
        raise NotImplementedError

    def get_car_pose(self, odometry: Odometry):
        """Return the current car pose

        Args:
            odometry: Odometry from nav_msgs.ms

        Returns:
            pose: np array [x, y, heading]
        """
        return utils.pose_to_particle(odometry.pose.pose)
    
    def _get_error_theta(self, pose: np.ndarray, target: np.ndarray) -> np.float:
        return self._wrap_radians_angle(pose[2] - target[2])

    def _path_complete_theta(self, error: np.float) -> bool:
        return np.linalg.norm(error) < self.finish_threshold_theta

    def _get_control_theta(self, error: np.float) -> np.ndarray:
        return np.array([0, -self.kw * error])

    def callback(self, odometry: Odometry):
        self.curr_pose = np.array(self.get_car_pose(odometry))
        if DEBUG:
            rospy.loginfo(rospy.get_caller_id() + " Current pose: %s" %
                      np.array2string(self.curr_pose))

    def get_reference_index(self, pose: np.ndarray, path: list, gotopt: int):
        """Return the index to the next control waypoint on the path

        Args:
            pose: np.ndarray: current pose of the car [x, y, heading]
            path: list[np.ndarray] of control waypoints

        Returns:
            index to the next control waypoint on the reference path
        """
        target = np.array(path[gotopt])
        if np.linalg.norm(pose[:2] - target[:2]) < self.finish_threshold:
            rospy.loginfo(rospy.get_caller_id() + f": Reach waypoint {gotopt}")
            if self.is_xyt(target):
                self.control_loop_theta(target, gotopt)
            return gotopt + 1
        else:
            return gotopt

    def _no_progress(self) -> bool:
        """Returns true if there is no progress after 10 seconds"""
        if self.prev_pose is None:
            self.prev_pose = self.curr_pose
            self.prev_pose_stamp = time.time()
        else:
            progress = np.linalg.norm(self.curr_pose - self.prev_pose)
            if progress > self.progress_threshold:
                self.prev_pose = self.curr_pose
                self.prev_pose_stamp = time.time()
            elif time.time() - self.prev_pose_stamp > self.progress_time_threshold:
                return True
        return False

    def _path_complete(self, path: list, gotopt: np.int) -> bool:
        """Returns whether the reference path has been completed

        Args:
            path: list of pose [x, y, heading]
            error: current error [e_x, e_y, e_theta]

        Returns:
            True if the path has been completed
        """
        # return (self.get_reference_index(pose, self.path) == (len(self.path) - 1)) and (np.linalg.norm(error) < self.finish_threshold)
        return gotopt == len(path)

    def _publish_control(self, control: np.ndarray) -> None:
        """Publish a control input to stretch

        Args:
            control: np.ndarray: [v_x, w_z]

        Returns:
            publishes a Twist message 

        """
        control_cmd = Twist()
        control_cmd.linear.x = control[0]
        control_cmd.angular.z = control[1]
        self.pub.publish(control_cmd)

    def wrap_path_radians(self, path):
        for target in path:
            if len(target) >= 3:
                target[2] = self._wrap_radians_angle(target[2])

    def _wrap_radians_angle(self, angle):
        """Wraps the given angle to the range [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    # def set_target(self, x: np.float = None, y: np.float = None, t: np.float = None):
    #     self.is_xyt_controller = t is not None
    #     if self.is_xyt_controller:
    #         self.path = [np.array([x, y, t])]  # pose = [x, y, theta]
    #     else:
    #         self.path = [np.array([x, y])]

    def set_path(self, path: list):
        assert len(path) > 0, "Path should not be empty"
        self.path = np.array(path)

    def is_xyt(self, point: np.ndarray) -> bool:
        return len(point) == 3


if __name__ == '__main__':
    raise NotImplementedError(
        "Should not call the base controller. Should initialize the subclass controller and call the control loop.")
