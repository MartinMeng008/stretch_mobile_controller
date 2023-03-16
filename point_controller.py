#!/usr/bin/env python

import rospy
import numpy as np
import argparse
import sys
from stretch_controller.base_controller import BaseController

sys.path.insert(0, '/home/qian/catkin_ws/src/stretch_controller/scripts')

from tools import print_debug

DEBUG = False

class PointController(BaseController):
    def __init__(self, x: np.float = None, y: np.float = None, t: np.float = None):
        super().__init__(x, y, t)
        self.set_path([[0,2,0], [2,2,5/4*np.pi], [0,0,0], [2,0, 1/2*np.pi], [2,2,5/4*np.pi], [0,0,0]])
        

    def _get_control(self, pose: np.ndarray, target: np.ndarray, error: np.ndarray) -> np.ndarray:
        """
        Args:
            pose: [x, y, theta]
            target: [x, y, theta]
            error: [x, y]
        """
        v = self.kv * np.sqrt((np.square(error[0])) + (np.square(error[1])))
        angle = self._wrap_radians_angle(
            (np.arctan2(-error[1], -error[0]) - pose[2]))
        if DEBUG:
            rospy.loginfo(rospy.get_caller_id() +
                      ": Angle difference: %s" % np.array2string(angle))
            rospy.loginfo(rospy.get_caller_id() +
                      ": Angle difference: %s" % np.array2string(np.degrees(angle)))
        w = self.kw * angle
        return np.array([v, w])

    def _get_error(self, pose: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Returns errors in x, y
        Args:
            pose: [x, y ,theta]
            target: [x, y, theta]

        Returns:
            error: [x, y]
        """
        return pose[:2] - target[:2]

    # def _get_error_theta(self, pose: np.ndarray, target: np.ndarray) -> np.float:
    #     return pose[2] - target[2]

    # def _path_complete_theta(self, error: np.float) -> bool:
    #     return np.linalg.norm(error) < self.finish_threshold_theta

    # def _get_control_theta(self, error: np.float) -> np.ndarray:
    #     return np.array([0, -self.kw * error])

    # #### Control loop ####
    # def control_loop(self) -> None:
    #     rospy.loginfo(rospy.get_caller_id() + " Start control loop")
    #     self.wrap_path_radians(self.path)
    #     while not rospy.is_shutdown():
    #         if self.curr_pose is None:
    #             continue
    #         pose = self.curr_pose
    #         if self._no_progress():
    #             rospy.logerr(rospy.get_caller_id(
    #             ) + " Error: No progress after 10s -- Current pose: %s" % np.array2string(pose))
    #             break
    #         index = self.get_reference_index(pose, self.path)
    #         self.goal_pose = self.path[index]
    #         self.error = self._get_error(pose, self.goal_pose)
    #         if self._path_complete(pose, self.error):
    #             rospy.loginfo(rospy.get_caller_id() + " -- Path completed -- Current pose: %s -- Goal pose: %s" %
    #                           (np.array2string(pose), np.array2string(self.goal_pose)))
    #             if self.is_xyt_controller:
    #                 self.control_loop_theta(self.goal_pose)
    #             break
    #         self.next_control = self._get_control(
    #             pose, self.goal_pose, self.error)
    #         if DEBUG:
    #             rospy.loginfo(rospy.get_caller_id() + " Control: %s" %
    #                       np.array2string(self.next_control))
    #         self._publish_control(self.next_control)
    #         self.rate.sleep()

     #### Control loop ####
    # def control_loop(self):
    #     rospy.logdebug(rospy.get_caller_id() + " Start control loop")
    #     self.wrap_path_radians(self.path)
    #     gotopt = 0
    #     while not rospy.is_shutdown():
    #         if self.curr_pose is None:
    #             continue
    #         pose = self.curr_pose
    #         if self._path_complete(self.path, gotopt):
    #             rospy.loginfo(rospy.get_caller_id() + " Reach the end of path.")
    #             break
    #         if self._no_progress():
    #             rospy.logerr(rospy.get_caller_id(
    #             ) + " Error: No progress after 10s -- Current pose: %s" % np.array2string(pose))

    #             rospy.loginfo(rospy.get_caller_id() + " -- Path completed -- Current pose: %s -- Goal pose: %s" %
    #                           (np.array2string(pose), np.array2string(self.goal_pose)))
    #             break
    #         # gotopt = self.get_reference_index(pose, self.path, gotopt)
    #         if np.linalg.norm(pose - self.path[gotopt]) < self.finish_threshold:
    #             rospy.loginfo(rospy.get_caller_id() + f": Reach waypoint {gotopt}")

    #         self.goal_pose = self.path[gotopt]
    #         self.error = self._get_error(pose, self.goal_pose)
    #         self.next_control = self._get_control(
    #             pose, self.goal_pose, self.error)
    #         if DEBUG:
    #             rospy.logdebug(rospy.get_caller_id() + "Control: %s" %
    #                        np.array2string(self.next_control))
    #         self._publish_control(self.next_control)
    #         self.rate.sleep()


    # def control_loop_theta(self, target: np.ndarray) -> None:
    #     """After reaching x, y, rotate theta to the target location"""
    #     rospy.loginfo(rospy.get_caller_id() + " Start theta control loop")
    #     while not rospy.is_shutdown():
    #         pose = self.curr_pose
    #         if self._no_progress():
    #             rospy.logerr(rospy.get_caller_id(
    #             ) + " Error: No progress after 10s -- Current pose: %s" % np.array2string(pose))
    #             break
    #         self.error = self._get_error_theta(pose, target)
    #         if self._path_complete_theta(self.error):
    #             rospy.loginfo(rospy.get_caller_id() + " -- Path completed -- Current pose: %s -- Goal pose: %s" %
    #                           (np.array2string(pose), np.array2string(target)))
    #             break
    #         self.next_control = self._get_control_theta(self.error)
    #         if DEBUG:
    #             rospy.loginfo(rospy.get_caller_id() + " Control: %s" %
    #                       np.array2string(self.next_control))
    #         self._publish_control(self.next_control)
    #         self.rate.sleep()

    # def _path_complete(self, pose: list, error: np.ndarray) -> bool:
    #     """Returns whether the reference path has been completed

    #     Args:
    #         pose: list of pose [x, y, heading]
    #         error: current error [e_x, e_y, e_theta]

    #     Returns:
    #         True if the path has been completed
    #     """
    #     return (self.get_reference_index(pose, self.path) == (len(self.path) - 1)) and (np.linalg.norm(error) < self.finish_threshold)
    #     # return gotopt == len(path)



if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            prog='StretchPointController', description='Control the stretch robot to move to a user-provided target pose [x, y, theta]')
        parser.add_argument('-x', action='store', dest='x', default='-3')
        parser.add_argument('-y', action='store', dest='y', default='0')
        parser.add_argument('-t', action='store', dest='t', default='0')
        args = parser.parse_args()
        controller = PointController(
            np.float(args.x), np.float(args.y), np.float(args.t))
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
