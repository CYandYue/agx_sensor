#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

// Every point is transformed at its own acquisition time into the optical frame
// at the RGB query time. Trajectory interpolation is an estimate, not hardware sync.
class Projector {
  using Cloud = livox_ros_driver2::CustomMsg;
  using Clock = std::chrono::steady_clock;
  struct Pending { sensor_msgs::Image::ConstPtr rgb; Clock::time_point arrival; };
  ros::NodeHandle nh_, pnh_{"~"};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_{tf_};
  std::vector<ros::Subscriber> subs_;
  ros::Publisher rgb_pub_, depth_pub_, ci_pub_, di_pub_, mask_pub_, pose_pub_, meta_pub_;
  ros::WallTimer timer_;
  using Scan = std::pair<Cloud::ConstPtr,double>;
  std::array<std::deque<Scan>,2> clouds_;
  std::deque<nav_msgs::Odometry::ConstPtr> odoms_;
  std::deque<sensor_msgs::CameraInfo::ConstPtr> infos_;
  std::deque<Pending> images_;
  std::string world_, body_, optical_;
  double half_window_, offset_, max_gap_, wait_, rate_, next_due_=0;
  uint64_t frames_=0, dropped_=0;
  std::array<Eigen::Isometry3d,3> accepted_ext_;
  bool have_ext_=false;

  static double seconds(const Clock::time_point& t) {
    return std::chrono::duration<double>(Clock::now()-t).count();
  }
  static Eigen::Isometry3d matrix(const geometry_msgs::Pose& p) {
    Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
    Eigen::Vector3d xyz(p.position.x,p.position.y,p.position.z);
    if(!q.coeffs().allFinite() || !xyz.allFinite() || std::abs(q.norm()-1)>.01)
      throw std::runtime_error("Invalid odometry pose");
    Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
    t.linear()=q.normalized().toRotationMatrix(); t.translation()=xyz; return t;
  }
  bool pose(double t, Eigen::Isometry3d& out) const {
    if(odoms_.size()<2) return false;
    auto b=std::lower_bound(odoms_.begin(),odoms_.end(),t,
      [](const nav_msgs::Odometry::ConstPtr& m,double v){return m->header.stamp.toSec()<v;});
    if(b==odoms_.begin() && t==(*b)->header.stamp.toSec()) ++b;
    if(b==odoms_.begin() || b==odoms_.end()) return false;
    auto a=b-1; double ta=(*a)->header.stamp.toSec(), tb=(*b)->header.stamp.toSec();
    if(tb-ta>max_gap_ || tb<=ta) return false;
    double alpha=(t-ta)/(tb-ta);
    Eigen::Isometry3d ma=matrix((*a)->pose.pose),mb=matrix((*b)->pose.pose);
    out=Eigen::Isometry3d::Identity();
    out.linear()=Eigen::Quaterniond(ma.linear()).slerp(alpha,Eigen::Quaterniond(mb.linear())).toRotationMatrix();
    out.translation()=(1-alpha)*ma.translation()+alpha*mb.translation(); return true;
  }
  Eigen::Isometry3d ext(const std::string& frame) {
    return tf2::transformToEigen(tf_.lookupTransform(body_,frame,ros::Time(0)));
  }
  void fail(const std::string& s) { ROS_FATAL_STREAM(s); ros::shutdown(); }
  bool render(const Pending& pending) {
    const auto& rgb=*pending.rgb; double tr=rgb.header.stamp.toSec(), target=tr+offset_;
    if(rgb.header.frame_id!=optical_) throw std::runtime_error("RGB optical frame mismatch");
    sensor_msgs::CameraInfo::ConstPtr ci;
    for(const auto& m:infos_) if(m->header.stamp.toSec()<=tr+.001) ci=m;
    if(!ci || odoms_.empty()) return false;
    if(ci->width!=rgb.width || ci->height!=rgb.height || ci->header.frame_id!=optical_ ||
       ci->K[0]<=0 || ci->K[4]<=0 || ci->binning_x>1 || ci->binning_y>1 ||
       ci->roi.width || ci->roi.height || ci->roi.x_offset || ci->roi.y_offset)
      throw std::runtime_error("Unsupported CameraInfo geometry/ROI/binning");
    for(int i=0;i<9;++i) if(!std::isfinite(ci->K[i]) || !std::isfinite(ci->R[i]) ||
       std::abs(ci->R[i]-(i%4==0?1.:0.))>1e-6) throw std::runtime_error("Invalid CameraInfo K/R");
    for(double d:ci->D) if(!std::isfinite(d)) throw std::runtime_error("Invalid distortion coefficients");
    if(ci->distortion_model!="plumb_bob" && ci->distortion_model!="rational_polynomial" &&
       !(ci->distortion_model=="equidistant" && ci->D.size()==4))
      throw std::runtime_error("Unsupported camera distortion model");
    for(const auto& stream:clouds_) {
      if(stream.empty()) return false;
      const auto& last=*stream.back().first;
      if(last.points.empty() || stream.back().second < target+half_window_)
        return false;  // Wait for a complete, symmetric acquisition window.
    }
    Eigen::Isometry3d wb, dummy;
    if(!pose(target,wb) || !pose(target-half_window_,dummy) || !pose(target+half_window_,dummy)) return false;
    std::array<Eigen::Isometry3d,3> ex;
    try { ex={{ext(optical_),ext("mid360_a_lidar"),ext("mid360_b_lidar")}}; }
    catch(const tf2::TransformException&) { return false; }
    if(have_ext_) for(int i=0;i<3;++i)
      if(!ex[i].matrix().isApprox(accepted_ext_[i].matrix(),1e-8))
        throw std::runtime_error("Sensor mounting extrinsics changed; start a new session");
    accepted_ext_=ex; have_ext_=true;
    Eigen::Isometry3d wc=wb*ex[0], cw=wc.inverse();
    std::vector<cv::Point3d> xyz;
    std::array<int,2> scans{{0,0}}, selected{{0,0}};
    std::array<std::vector<uint64_t>,2> scan_stamps;
    double first=std::numeric_limits<double>::infinity(),last=-first;
    double scan_first=first,scan_last=last;
    for(int device=0;device<2;++device) for(const auto& scan:clouds_[device]) {
      const auto& msg=scan.first;
      double start=msg->header.stamp.toSec();
      if(msg->points.empty() || start>target+half_window_ ||
         scan.second<target-half_window_) continue;
      ++scans[device]; scan_stamps[device].push_back(msg->header.stamp.toNSec()); scan_first=std::min(scan_first,start); scan_last=std::max(scan_last,start);
      for(const auto& p:msg->points) {
        double tp=start+p.offset_time*1e-9;
        if(tp<target-half_window_ || tp>target+half_window_ ||
           !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
           ((p.tag&0x30)!=0 && (p.tag&0x30)!=0x10) ||
           p.x*p.x+p.y*p.y+p.z*p.z<.25) continue;
        Eigen::Isometry3d wp;
        if(!pose(tp,wp)) return false; // Never substitute a stale pose or extrapolate.
        Eigen::Vector3d pc=cw*wp*ex[device+1]*Eigen::Vector3d(p.x,p.y,p.z);
        if(pc.z()<=.1 || pc.z()>60 || !pc.allFinite()) continue;
        xyz.emplace_back(pc.x(),pc.y(),pc.z()); ++selected[device];
        first=std::min(first,tp);last=std::max(last,tp);
      }
    }
    if(scans[0]==0 || scans[1]==0 || xyz.empty()) return false;
    cv::Mat k(3,3,CV_64F),d(1,ci->D.size(),CV_64F);
    for(int i=0;i<9;++i) k.at<double>(i/3,i%3)=ci->K[i];
    for(size_t i=0;i<ci->D.size();++i) d.at<double>(0,i)=ci->D[i];
    std::vector<cv::Point2d> pixels;
    if(ci->distortion_model=="equidistant")
      cv::fisheye::projectPoints(xyz,pixels,cv::Vec3d(),cv::Vec3d(),k,d);
    else cv::projectPoints(xyz,cv::Vec3d(),cv::Vec3d(),k,d,pixels);
    std::vector<float> z(rgb.width*rgb.height,std::numeric_limits<float>::quiet_NaN());
    for(size_t i=0;i<pixels.size();++i) {
      double x=pixels[i].x,y=pixels[i].y;
      if(!std::isfinite(x) || !std::isfinite(y) || x<-.5 || y<-.5 ||
         x>=rgb.width-.5 || y>=rgb.height-.5) continue;
      int u=std::floor(x+.5),v=std::floor(y+.5); auto index=v*rgb.width+u;
      if(!std::isfinite(z[index]) || xyz[i].z<z[index]) z[index]=xyz[i].z;
    }
    sensor_msgs::Image depth,mask;
    depth.header=mask.header=rgb.header;
    depth.width=mask.width=rgb.width;depth.height=mask.height=rgb.height;
    depth.encoding="32FC1";depth.is_bigendian=false;depth.step=rgb.width*4;
    depth.data.resize(z.size()*4);std::memcpy(depth.data.data(),z.data(),depth.data.size());
    mask.encoding="mono8";mask.step=rgb.width;mask.data.resize(z.size());
    size_t valid=0;for(size_t i=0;i<z.size();++i) {mask.data[i]=std::isfinite(z[i])?255:0;valid+=mask.data[i]!=0;}
    if(!valid) return false;
    geometry_msgs::PoseStamped camera_pose;camera_pose.header=rgb.header;camera_pose.header.frame_id=world_;
    camera_pose.pose.position.x=wc.translation().x();camera_pose.pose.position.y=wc.translation().y();camera_pose.pose.position.z=wc.translation().z();
    Eigen::Quaterniond q(wc.linear());camera_pose.pose.orientation.x=q.x();camera_pose.pose.orientation.y=q.y();
    camera_pose.pose.orientation.z=q.z();camera_pose.pose.orientation.w=q.w();
    std_msgs::String meta;std::ostringstream s;s<<std::setprecision(17);
    s<<"{\"rgb_stamp_ns\":"<<rgb.header.stamp.toNSec()<<",\"pose_query_stamp_ns\":"<<(rgb.header.stamp.toNSec()+int64_t(std::llround(offset_*1e9)))
     <<",\"point_first_stamp_ns\":"<<ros::Time(first).toNSec()<<",\"point_last_stamp_ns\":"<<ros::Time(last).toNSec()
     <<",\"point_half_window_s\":"<<half_window_<<",\"source_scan_first_ns\":"<<ros::Time(scan_first).toNSec()
     <<",\"source_scan_last_ns\":"<<ros::Time(scan_last).toNSec()<<",\"source_scan_counts\":["<<scans[0]<<","<<scans[1]
     <<"],\"selected_point_counts\":["<<selected[0]<<","<<selected[1]<<"],\"valid_pixels\":"<<valid
     <<",\"valid_fraction\":"<<double(valid)/z.size()<<",\"output_age_s\":"<<(ros::Time::now().toSec()-tr)
     <<",\"motion_model\":\"linear_translation_slerp\",\"depth_source\":\"mid360\",\"hardware_sync_verified\":false,\"accuracy_validated\":false}";
    std::string json=s.str(); json.pop_back();s.str("");s.clear();s<<json<<",\"source_scan_stamps_ns\":[";
    for(int i=0;i<2;++i) {if(i)s<<",";s<<"[";for(size_t j=0;j<scan_stamps[i].size();++j){if(j)s<<",";s<<scan_stamps[i][j];}s<<"]";}s<<"]}";
    meta.data=s.str();
    auto info=*ci;info.header=rgb.header;
    meta_pub_.publish(meta);ci_pub_.publish(info);di_pub_.publish(info);
    rgb_pub_.publish(pending.rgb);depth_pub_.publish(depth);mask_pub_.publish(mask);pose_pub_.publish(camera_pose);
    ++frames_;ROS_INFO_STREAM_THROTTLE(5,"frames="<<frames_<<" valid="<<double(valid)/z.size()<<" age="<<ros::Time::now().toSec()-tr<<"s dropped="<<dropped_);
    return true;
  }
  void tick(const ros::WallTimerEvent&) {
    try {
      while(!images_.empty()) {
        if(render(images_.front())) images_.pop_front();
        else if(seconds(images_.front().arrival)>wait_) {++dropped_; images_.pop_front(); ROS_WARN_THROTTLE(5,"Projection timed out waiting for clouds/odometry/calibration; frame skipped");}
        else break;
      }
    } catch(const std::exception& e) { fail(e.what()); }
  }
public:
  Projector() {
    pnh_.param<std::string>("world",world_,"lio_map");pnh_.param<std::string>("body",body_,"body");
    pnh_.param<std::string>("optical",optical_,"camera_color_optical_frame");
    pnh_.param("point_half_window",half_window_,.05);pnh_.param("camera_time_offset",offset_,0.);
    pnh_.param("max_odom_gap",max_gap_,.25);pnh_.param("wait",wait_,3.);pnh_.param("rate",rate_,6.);
    if(!std::isfinite(half_window_) || half_window_<=0 || half_window_>.2 || !std::isfinite(offset_) ||
       !std::isfinite(rate_) || rate_<=0 || rate_>10 || !std::isfinite(max_gap_) || !std::isfinite(wait_) || max_gap_<=0 || max_gap_>1 || wait_<=0 || wait_>8)
      throw std::runtime_error("Invalid projection parameters");
    if(nh_.param("/lio/mapping/extrinsic_est_en",false)) throw std::runtime_error("Online extrinsic estimation is unsupported");
    auto output=nh_.resolveName("/lidar_rgbd");
    rgb_pub_=nh_.advertise<sensor_msgs::Image>(output+"/rgb/image_raw",3);
    depth_pub_=nh_.advertise<sensor_msgs::Image>(output+"/depth/image_raw",3);
    ci_pub_=nh_.advertise<sensor_msgs::CameraInfo>(output+"/rgb/camera_info",3);
    di_pub_=nh_.advertise<sensor_msgs::CameraInfo>(output+"/depth/camera_info",3);
    mask_pub_=nh_.advertise<sensor_msgs::Image>(output+"/valid_mask",3);
    pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>(output+"/camera_pose",3);
    meta_pub_=nh_.advertise<std_msgs::String>(output+"/projection",20);
    subs_.push_back(nh_.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info",60,[this](const sensor_msgs::CameraInfo::ConstPtr& m){infos_.push_back(m);while(infos_.size()>240)infos_.pop_front();}));
    subs_.push_back(nh_.subscribe<sensor_msgs::Image>("/camera/color/image_raw",6,[this](const sensor_msgs::Image::ConstPtr& m){
      if(m->header.stamp.isZero()) {fail("RGB has zero stamp");return;}
      static double previous=0;double t=m->header.stamp.toSec();
      if(t<previous) {fail("RGB clock moved backward");return;} if(t==previous)return;previous=t;
      if(next_due_==0)next_due_=t;
      if(t+.001<next_due_)return;next_due_+=(std::floor(std::max(0.,t-next_due_)*rate_)+1)/rate_;
      if(images_.size()>=48) {fail("Image processing queue overflow");return;}
      images_.push_back({m,Clock::now()});}));
    subs_.push_back(nh_.subscribe<nav_msgs::Odometry>("/lio/odometry",300,[this](const nav_msgs::Odometry::ConstPtr& m){
      if(m->header.frame_id!=world_ || m->child_frame_id!=body_) {fail("Odometry frame changed");return;}
      if(!odoms_.empty() && m->header.stamp<=odoms_.back()->header.stamp) {fail("Odometry clock non-increasing");return;}
      odoms_.push_back(m);while(odoms_.size()>2 && (odoms_.back()->header.stamp-odoms_[1]->header.stamp).toSec()>12) odoms_.pop_front();
    }));
    for(int i=0;i<2;++i) {
      std::string topic=i==0?"/livox/lidar_192_168_1_113":"/livox/lidar_192_168_1_154";
      subs_.push_back(nh_.subscribe<Cloud>(topic,30,[this,i](const Cloud::ConstPtr& m){
        auto& stream=clouds_[i];
        if(m->header.stamp.isZero() || m->point_num!=m->points.size() || m->points.empty()) {fail("Invalid raw lidar message");return;}
        if(!stream.empty() && m->header.stamp<=stream.back().first->header.stamp) {fail("Lidar clock non-increasing");return;}
        uint32_t end_offset=0;
        for(const auto& point:m->points)end_offset=std::max(end_offset,point.offset_time);
        if(end_offset>300000000) {fail("Unexpected lidar scan duration");return;}
        stream.emplace_back(m,m->header.stamp.toSec()+end_offset*1e-9);
        while(stream.size()>100)stream.pop_front();
      },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
    }
    timer_=nh_.createWallTimer(ros::WallDuration(.02),&Projector::tick,this);
    ROS_WARN("Sparse lidar depth; mounting/time accuracy unverified. Stationary-scene motion compensation uses interpolated LIO poses.");
  }
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"lidar_depth");
  try {Projector projector;ros::spin();}catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 1;}
  return ros::ok()?0:0;
}
