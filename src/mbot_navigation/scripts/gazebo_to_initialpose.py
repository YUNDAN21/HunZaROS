#!/usr/bin/env python3
"""
简单来说，就是把Gazebo中小车位置读出，并发布到 /initialpose，让AMCL（或其他定位节点）使用Gazebo位置作为初始位置。
gazebo_to_initialpose.py

Read a model's pose from Gazebo and publish it once to /initialpose
so AMCL (or other localization node) uses the Gazebo pose as initial pose.

Usage (with rosrun or roslaunch):
rosrun mbot_navigation gazebo_to_initialpose.py _model_name:=limo/ _frame:=map

Parameters (rosparam / private args):
  ~model_name   (string) model name in Gazebo (default: 'limo/')
  ~frame        (string) frame id to publish in header (default: 'map')
  ~covariance   (list[36]) optional covariance matrix (row-major)

This script waits for the /gazebo/get_model_state service, requests the
pose for the given model, and publishes a latched PoseWithCovarianceStamped
to /initialpose.
"""

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys


def default_covariance():
    # Use reasonable defaults: small uncertainty in x,y and yaw, large for others
    cov = [0.0] * 36
    cov[0] = 0.25   # x
    cov[7] = 0.25   # y
    cov[14] = 99999.0  # z (unused)
    cov[21] = 99999.0  # rot_x
    cov[28] = 99999.0  # rot_y
    cov[35] = 0.35   # rot_z (yaw)
    return cov


def main():
    rospy.init_node('gazebo_to_initialpose')

    model_name = rospy.get_param('~model_name', 'limo/')
    frame_id = rospy.get_param('~frame', 'map')
    cov_param = rospy.get_param('~covariance', None)

    if cov_param is None:
        covariance = default_covariance()
    else:
        try:
            if len(cov_param) != 36:
                rospy.logerr('Parameter ~covariance must be length 36 (row-major).')
                return 1
            covariance = [float(x) for x in cov_param]
        except Exception as e:
            rospy.logerr('Invalid covariance param: %s', e)
            return 1

    rospy.loginfo('Waiting for /gazebo/get_model_state service...')
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=10)
    except rospy.ROSException:
        rospy.logerr('Service /gazebo/get_model_state not available. Is Gazebo running?')
        return 1

    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name, '')
    except Exception as e:
        rospy.logerr('Failed to call get_model_state: %s', e)
        return 1

    if not resp.success:
        rospy.logerr('Gazebo returned success=False for model "%s"', model_name)
        return 1

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.pose = resp.pose
    msg.pose.covariance = covariance

    # Wait a short moment for publisher registration
    rospy.sleep(0.2)
    pub.publish(msg)
    rospy.loginfo('Published /initialpose from Gazebo model "%s" to frame "%s"', model_name, frame_id)

    # keep node alive briefly so latched msg remains available to late subscribers
    rospy.sleep(1.0)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main() or 0)
    except rospy.ROSInterruptException:
        pass
