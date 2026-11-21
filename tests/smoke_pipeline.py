#!/usr/bin/env python3
"""Private ROS master, real C++ projector and recorder. --hardware starts sensors only."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'recorder'))
import numpy as np
import rosbag
from capture import check_bag, frame_groups, GROUP
from core import pose_matrix, image_array


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--duration',type=float,default=8)
    args=parser.parse_args()
    output=Path(tempfile.mkdtemp(prefix='lidar-rgbd-hardware-' if args.hardware else 'lidar-rgbd-synthetic-'))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(output/'ros'))
    os.environ.pop('ROS_HOSTNAME',None)
    processes=[];handles=[]
    def start(cmd,name):
        f=(output/name).open('w');handles.append(f)
        process=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    try:
        start(['roscore','-p',str(port)],'master.log')
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(150):
            try:
                if master.getPid('/smoke')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('Private ROS master did not start')
        import rospy
        from sensor_msgs.msg import CameraInfo,Image,Imu
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from livox_ros_driver2.msg import CustomMsg,CustomPoint
        from tf.transformations import quaternion_from_matrix
        rospy.init_node('pipeline_fixture',disable_signals=True)
        start(['roslaunch','lidar_rgbd_capture','sensors.launch' if args.hardware else 'projector.launch'],'sensors.log')
        if args.hardware:
            # Drivers and LIO initialize before the recording duration starts.
            time.sleep(12)
        else:
            pubs={t:rospy.Publisher(t,cls,queue_size=50,latch=t in ('/tf_static','/camera/color/camera_info')) for t,cls in [
                ('/camera/color/image_raw',Image),('/camera/color/camera_info',CameraInfo),
                ('/lio/odometry',Odometry),('/tf',TFMessage),('/tf_static',TFMessage),
                ('/livox/lidar_192_168_1_113',CustomMsg),('/livox/lidar_192_168_1_154',CustomMsg),
                ('/livox/imu_192_168_1_113',Imu),('/livox/imu_192_168_1_154',Imu),('/imu/data_raw',Imu)]}
            ci=CameraInfo();ci.width=64;ci.height=48;ci.header.frame_id='camera_color_optical_frame'
            ci.K=[50.,0,31.5,0,50.,23.5,0,0,1];ci.P=[50.,0,31.5,0,0,50.,23.5,0,0,0,1,0]
            ci.R=list(np.eye(3).flat);ci.D=[.08,-.01,.001,0,0];ci.distortion_model='plumb_bob'
            static=[]
            for child,x in [('camera_color_optical_frame',.1),('mid360_a_lidar',.2),('mid360_b_lidar',-.2)]:
                t=TransformStamped();t.header.frame_id='body';t.child_frame_id=child;t.transform.translation.x=x;t.transform.rotation.w=1;static.append(t)
            pubs['/tf_static'].publish(TFMessage(static));pubs['/camera/color/camera_info'].publish(ci)
            epoch=rospy.Time.now().to_sec()
            world=np.array([(x,y,z) for z in (3.,5.) for x in np.linspace(-1,1,13) for y in np.linspace(-.7,.7,11)])
            def body(t):
                dt=t-epoch;angle=.03*dt;c=np.cos(angle);s=np.sin(angle)
                m=np.eye(4);m[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]];m[:3,3]=[.05*dt,0,0];return m
            def publish(i):
                now=rospy.Time.now();t=now.to_sec()
                m=Odometry();m.header.stamp=now;m.header.frame_id='lio_map';m.child_frame_id='body'
                b=body(t);q=quaternion_from_matrix(b)
                m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=b[:3,3]
                m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=q
                pubs['/lio/odometry'].publish(m)
                tr=TransformStamped();tr.header=m.header;tr.child_frame_id='body';tr.transform.translation.x=b[0,3];tr.transform.rotation=m.pose.pose.orientation
                pubs['/tf'].publish(TFMessage([tr]))
                imu=Imu();imu.header.stamp=now;imu.orientation.w=1;imu.linear_acceleration.z=9.8
                for topic in ('/imu/data_raw','/livox/imu_192_168_1_113','/livox/imu_192_168_1_154'):pubs[topic].publish(imu)
                if i%5==0:
                    for ip,x,skew in [('113',.2,0.),('154',-.2,-.02)]:
                        cloud=CustomMsg();start_time=t-.1+skew;cloud.header.stamp=rospy.Time.from_sec(start_time);cloud.header.frame_id='livox_frame'
                        offsets=np.rint(np.arange(len(world))*99000000/(len(world)-1)).astype(int)
                        dt=start_time+offsets*1e-9-epoch;c=np.cos(.03*dt);s=np.sin(.03*dt)
                        dx=world[:,0]-.05*dt
                        local=np.column_stack((c*dx-s*world[:,2]-x,world[:,1],s*dx+c*world[:,2]))
                        for j,xyz in enumerate(local):
                            p=CustomPoint();p.offset_time=int(offsets[j]);p.tag=0x10;p.line=j%4
                            p.x,p.y,p.z=xyz;p.reflectivity=80;cloud.points.append(p)
                        cloud.points.reverse() # Raw drivers need not order points by acquisition time.
                        cloud.point_num=len(cloud.points);pubs['/livox/lidar_192_168_1_'+ip].publish(cloud)
                if i%8==0 and i>25:
                    image=Image();image.header.stamp=now;image.header.frame_id=ci.header.frame_id;image.header.seq=i//8
                    image.width=64;image.height=48;image.encoding='rgb8';image.step=64*3;image.data=bytes([80,120,160])*(64*48)
                    info=copy.deepcopy(ci);info.header=image.header
                    pubs['/camera/color/camera_info'].publish(info);pubs['/camera/color/image_raw'].publish(image)
        bag=output/'scene.bag'
        recorder=start([str(ROOT/'run.sh'),'record',str(bag),'--duration',str(args.duration),'--startup-timeout','30','--min-free-gb','0'],'record.log')
        began=time.monotonic();i=0
        while recorder.poll() is None and time.monotonic()-began<args.duration+45:
            if not args.hardware:publish(i)
            i+=1;time.sleep(max(0,began+i*.02-time.monotonic()))
        recorder.wait(timeout=5)
        if recorder.returncode:raise RuntimeError('Recording failed: '+str(output/'record.log'))
        result=check_bag(bag)
        result['hardware']=args.hardware;result['bag_bytes']=bag.stat().st_size
        result['log_directory']=str(output)
        fractions=[];ages=[];error=0.;pose_error=0.;checked=0;best=None;best_fraction=0
        with rosbag.Bag(str(bag)) as data:
            raw=list(data.read_messages(topics=['/livox/lidar_192_168_1_113','/livox/lidar_192_168_1_154'])) if not args.hardware else []
            for ns,g in frame_groups(data):
                meta=json.loads(g[GROUP[-1]].data);fractions.append(meta['depth_valid_fraction']);ages.append(meta['projection']['output_age_s'])
                if meta['depth_valid_fraction']>best_fraction:
                    best_fraction=meta['depth_valid_fraction'];best=(np.array(image_array(g[GROUP[0]])),np.array(image_array(g[GROUP[1]])),g[GROUP[0]].encoding)
                if args.hardware:continue
                import cv2
                wc=body(ns/1e9);wc[:3,3]+=wc[:3,:3]@np.array([.1,0,0])
                pose_error=max(pose_error,float(np.max(np.abs(pose_matrix(g[GROUP[4]].pose)-wc))))
                selected=[]
                for _,cloud,_ in raw:
                    point_times=cloud.header.stamp.to_sec()+np.array([p.offset_time for p in cloud.points])*1e-9
                    use=np.abs(point_times-ns/1e9)<=.05
                    if use.any():selected.append(world[::-1][use])
                sampled=np.concatenate(selected)
                pc=(wc[:3,:3].T@(sampled-wc[:3,3]).T).T
                pixels,_=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),np.array(ci.K).reshape(3,3),np.array(ci.D))
                expected=np.full((48,64),np.nan,dtype=float)
                for uv,p in zip(pixels[:,0],pc):
                    u,v=np.floor(uv+.5).astype(int)
                    if p[2]>0 and 0<=u<64 and 0<=v<48 and (np.isnan(expected[v,u]) or p[2]<expected[v,u]):expected[v,u]=p[2]
                depth=image_array(g[GROUP[1]]);valid=np.isfinite(depth)
                assert np.isfinite(expected[valid]).all(),'Projection landed on an analytically incorrect pixel'
                error=max(error,float(np.max(np.abs(depth[valid]-expected[valid]))));checked+=int(valid.sum())
        result.update(mean_valid_fraction=float(np.mean(fractions)),mean_output_age_s=float(np.mean(ages)),max_output_age_s=float(max(ages)))
        if not args.hardware:
            result.update(analytic_depth_max_error_m=error,analytic_pose_matrix_max_error=pose_error,checked_depth_pixels=checked)
            assert result['frames']>=15,result
            assert error<1e-4,error
            assert pose_error<1e-5,pose_error
            result.update(analytic_depth_max_error_m=error,analytic_pose_matrix_max_error=pose_error,checked_depth_pixels=checked)
        if best is not None:
            import cv2
            rgb,depth,encoding=best
            if encoding=='rgb8':rgb=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            good=np.isfinite(depth)&(np.nan_to_num(depth)>0);scale=np.nanpercentile(depth[good],95)
            color=cv2.applyColorMap(np.uint8(np.clip(np.nan_to_num(depth)/max(scale,.1),0,1)*255),cv2.COLORMAP_TURBO)
            overlay=rgb.copy();overlay[good]=color[good]
            cv2.imwrite(str(output/'rgb.png'),rgb);cv2.imwrite(str(output/'projection_overlay.png'),overlay)
            color[~good]=0;cv2.imwrite(str(output/'depth_preview.png'),color)
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for handle in handles:handle.close()
        print('Test output:',output)


if __name__=='__main__':main()
