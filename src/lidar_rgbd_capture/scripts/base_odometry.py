#!/usr/bin/env python3
"""Expose FAST-LIO's base_link pose without waiting for RGB or projecting depth.

The existing sensor_frames node owns body -> lidar -> base_link static TF.
Do not broadcast map -> base_link: that would give base_link a second parent.
"""
import copy
import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def pose_matrix(position, quaternion):
    xyz = np.asarray(position, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if not np.isfinite(xyz).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > .01:
        raise ValueError('Invalid position or quaternion')
    result = quaternion_matrix(q)
    result[:3, 3] = xyz
    return result


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def convert(body, body_to_base, covariance, source_order='rotation_position'):
    """Convert the deployed FAST-LIO covariance to world fixed-axis pose error.

    use-ikfom.hpp declares [position, right/body SO(3) error]. The deployed
    laserMapping_bundle.cpp swaps those blocks when filling the ROS message.
    Undo that swap, rotate attitude error to world, and account for the lever
    arm. Mounting uncertainty is unknown and is not included in this estimate.
    """
    covariance = np.asarray(covariance, dtype=float).reshape(6, 6)
    if source_order == 'rotation_position':
        order = [3, 4, 5, 0, 1, 2]
        covariance = covariance[np.ix_(order, order)]
    elif source_order != 'position_rotation':
        raise ValueError('Unknown source covariance block order')
    if not np.isfinite(covariance).all():
        raise ValueError('Non-finite source pose covariance')
    covariance = .5 * (covariance + covariance.T)
    if np.linalg.eigvalsh(covariance).min() < -1e-8:
        raise ValueError('Source pose covariance is not positive semidefinite')
    rotation = body[:3, :3]
    jacobian = np.eye(6)
    jacobian[:3, 3:] = -rotation @ skew(body_to_base[:3, 3])
    jacobian[3:, 3:] = rotation
    covariance = jacobian @ covariance @ jacobian.T
    if not covariance.any():
        covariance = np.eye(6) * 1e6  # Source has no covariance estimate.
    return body @ body_to_base, covariance


def difference(previous, current, dt):
    """Backward difference, interval-average twist expressed in current base."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Twist requires increasing source timestamps')
    linear = current[:3, :3].T @ (current[:3, 3] - previous[:3, 3]) / dt
    rotation = np.eye(4)
    rotation[:3, :3] = previous[:3, :3].T @ current[:3, :3]
    q = quaternion_from_matrix(rotation)
    if q[3] < 0:
        q = -q
    norm = np.linalg.norm(q[:3])
    angular = (2. * q[:3] if norm < 1e-10 else
               q[:3] * (2. * math.atan2(norm, q[3]) / norm)) / dt
    return linear, angular


class BaseOdometry:
    def __init__(self):
        self.max_age = rospy.get_param('~max_source_age', .8)
        self.max_gap = rospy.get_param('~max_twist_gap', .5)
        self.covariance_order = rospy.get_param('~source_covariance_order', 'rotation_position')
        if self.covariance_order not in ('rotation_position', 'position_rotation'):
            raise ValueError('Unknown source covariance block order')
        if any(not math.isfinite(value) or value <= 0 for value in (self.max_age, self.max_gap)):
            raise ValueError('Source age and twist gap must be positive finite values')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.extrinsic = None
        self.previous = None
        self.publisher = rospy.Publisher('/lio/base_odometry', Odometry, queue_size=2)
        self.pose_publisher = rospy.Publisher('/lio/base_pose', PoseStamped, queue_size=2)
        self.subscriber = rospy.Subscriber('/lio/odometry', Odometry, self.receive,
                                           queue_size=1, tcp_nodelay=True)
        rospy.loginfo('Publishing LIO base_link pose using existing static TF; no wheel odometry, no RGB wait')
        rospy.logwarn('Base pose inherits vendor/nominal mounting parameters; physical calibration is unverified')

    def receive(self, source):
        try:
            age = (rospy.Time.now() - source.header.stamp).to_sec()
            if source.header.stamp.to_nsec() <= 0 or not math.isfinite(age) or age < -.1 or age > self.max_age:
                raise ValueError('Stale/future/zero LIO timestamp; preserving source time, no extrapolation')
            if source.header.frame_id != 'lio_map' or source.child_frame_id != 'body':
                raise ValueError('Expected lio_map/body input frames')
            transform = self.buffer.lookup_transform('body', 'base_link', rospy.Time(0)).transform
            p, q = transform.translation, transform.rotation
            extra = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            if self.extrinsic is not None and not np.allclose(extra, self.extrinsic, atol=1e-8, rtol=0):
                rospy.logfatal('body -> base_link mounting changed; restart after checking sensor geometry')
                rospy.signal_shutdown('Mounting transform changed')
                return
            self.extrinsic = extra
            p, q = source.pose.pose.position, source.pose.pose.orientation
            body = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            base, covariance = convert(body, extra, source.pose.covariance, self.covariance_order)
            stamp = source.header.stamp.to_nsec()
            dt = None if self.previous is None else (stamp - self.previous[0]) / 1e9
            if dt is not None and dt == 0:
                return
            output = Odometry()
            output.header = copy.deepcopy(source.header)
            output.child_frame_id = 'base_link'
            p = output.pose.pose.position
            p.x, p.y, p.z = base[:3, 3]
            q = output.pose.pose.orientation
            q.x, q.y, q.z, q.w = quaternion_from_matrix(base)
            output.pose.covariance = covariance.ravel().tolist()
            # FAST-LIO leaves twist empty. Estimate from base poses, never copy
            # its default zero twist or substitute wheel velocities. Cross-time
            # filter covariance is unavailable, so twist uncertainty is unknown.
            output.twist.covariance = (np.eye(6) * 1e6).ravel().tolist()
            if dt is not None and 0 < dt <= self.max_gap:
                linear, angular = difference(self.previous[1], base, dt)
                output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = linear
                output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = angular
            # A backward clock is published once so controllers detect/reset it
            # and latch stop; the derivative restarts from this new epoch.
            self.previous = (stamp, base)
            pose = PoseStamped(header=copy.deepcopy(source.header), pose=copy.deepcopy(output.pose.pose))
            self.publisher.publish(output)
            self.pose_publisher.publish(pose)
        except (ValueError, tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(2., 'LIO base pose unavailable: %s', str(exc))


if __name__ == '__main__':
    rospy.init_node('lio_base_odometry')
    BaseOdometry()
    rospy.spin()
