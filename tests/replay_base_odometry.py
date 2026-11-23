#!/usr/bin/env python3
"""Replay existing LIO + static geometry into the base publisher; no hardware."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    args = parser.parse_args()
    odometry = [];static = {}
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, _ in bag.read_messages(topics=['/lio/odometry', '/tf_static']):
            if topic == '/lio/odometry':odometry.append(message)
            else:
                for tr in message.transforms:static[tr.child_frame_id] = tr
    if len(odometry)<20:raise RuntimeError('Need at least 20 LIO samples')
    output = Path(tempfile.mkdtemp(prefix='lio-base-replay-'))
    with socket.socket() as sock:sock.bind(('127.0.0.1', 0));port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port, ROS_IP='127.0.0.1', ROS_LOG_DIR=str(output/'ros'))
    os.environ.pop('ROS_HOSTNAME', None)
    processes = [];handles = []

    def start(command, name):
        log = (output/name).open('w');handles.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)

    try:
        start(['roscore', '-p', str(port)], 'master.log')
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(150):
            try:
                if master.getPid('/replay')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('ROS master did not start')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from tf.transformations import quaternion_matrix
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('base_replay_fixture', disable_signals=True)
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
        clock_pub.publish(Clock(odometry[0].header.stamp+rospy.Duration(.1)))
        source_pub = rospy.Publisher('/lio/odometry', Odometry, queue_size=1)
        results = []
        rospy.Subscriber('/lio/base_odometry', Odometry, results.append, queue_size=200)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(list(static.values()))
        buffer = tf2_ros.Buffer()
        for tr in static.values():
            # rosbag may use dynamically generated message classes. tf2's
            # Python binding expects the installed geometry message classes.
            normalized = TransformStamped()
            normalized.header.frame_id = tr.header.frame_id
            normalized.child_frame_id = tr.child_frame_id
            normalized.header.stamp = tr.header.stamp
            for axis in ('x', 'y', 'z'):
                setattr(normalized.transform.translation, axis, getattr(tr.transform.translation, axis))
            for axis in ('x', 'y', 'z', 'w'):
                setattr(normalized.transform.rotation, axis, getattr(tr.transform.rotation, axis))
            buffer.set_transform_static(normalized, 'recorded_geometry')
        extra = buffer.lookup_transform('body', 'base_link', rospy.Time(0)).transform

        def matrix(position, quaternion):
            result = quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            result[:3, 3] = [position.x, position.y, position.z]
            return result

        extra = matrix(extra.translation, extra.rotation)
        expected = {m.header.stamp.to_nsec():matrix(m.pose.pose.position, m.pose.pose.orientation)@extra for m in odometry}
        start(['roslaunch', 'lidar_rgbd_capture', 'base_odometry.launch'], 'base_odometry.log')
        deadline = time.monotonic()+12.
        while source_pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.05)
        if not source_pub.get_num_connections():raise RuntimeError('Base node did not subscribe')
        time.sleep(.3)
        for msg in odometry:
            clock_pub.publish(Clock(msg.header.stamp+rospy.Duration(.1)))
            source_pub.publish(msg)
            time.sleep(.04)
        time.sleep(.3)
        assert len(results)>=len(odometry)*.8, (len(results), len(odometry))
        maximum_error = 0.;minimum_eigenvalue = float('inf')
        for msg in results:
            assert msg.header.frame_id=='lio_map' and msg.child_frame_id=='base_link'
            assert msg.header.stamp.to_nsec() in expected,'Source timestamp was changed'
            actual = matrix(msg.pose.pose.position, msg.pose.pose.orientation)
            maximum_error = max(maximum_error, float(np.max(np.abs(actual-expected[msg.header.stamp.to_nsec()]))))
            covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
            assert np.isfinite(covariance).all()
            minimum_eigenvalue = min(minimum_eigenvalue, float(np.linalg.eigvalsh(covariance).min()))
        assert maximum_error<1e-8, maximum_error
        assert minimum_eigenvalue>=-1e-8, minimum_eigenvalue
        result = dict(status='PASS', hardware_opened=False, source_bag=str(args.bag.resolve()),
                      input_samples=len(odometry), output_samples=len(results),
                      analytic_base_pose_matrix_max_error=maximum_error,
                      pose_covariance_min_eigenvalue=minimum_eigenvalue,
                      source_timestamps_preserved=True, logs=str(output))
        (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:process.wait(timeout=6)
                except subprocess.TimeoutExpired:os.killpg(process.pid, signal.SIGTERM);process.wait(timeout=4)
        for handle in handles:handle.close()
        print('Test output:', output)


if __name__=='__main__':main()
