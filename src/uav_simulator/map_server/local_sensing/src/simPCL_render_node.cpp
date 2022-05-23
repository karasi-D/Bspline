#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Publisher pub_mv_obs, pub_mv_obs_traj;
geometry_msgs::Point MoveObs_pt;
visualization_msgs::Marker MoveObs_traj;
ros::Timer MvObs_update_timer;
double t_duration;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution, _mvObs_vel;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

void MoveObstaclesInit()
{
   MoveObs_pt.x = -10.0;
   MoveObs_pt.y = 20.0;
   MoveObs_pt.z = 2.0;
   
   MoveObs_traj.header.stamp       = ros::Time::now(); 
   MoveObs_traj.header.frame_id    = "world";

   MoveObs_traj.ns = "MoveObstacles/trajectory";
   MoveObs_traj.id = 0;
   MoveObs_traj.type = visualization_msgs::Marker::CUBE_LIST;
   // visualization_msgs::Marker::CUBE_LIST | SPHERE_LIST;
   MoveObs_traj.action = visualization_msgs::Marker::DELETE;

   MoveObs_traj.action = visualization_msgs::Marker::ADD;
   MoveObs_traj.scale.x = 0.4;
   MoveObs_traj.scale.y = 0.4;
   MoveObs_traj.scale.z = 0.4;
   MoveObs_traj.pose.orientation.x = 0.0;
   MoveObs_traj.pose.orientation.y = 0.0;
   MoveObs_traj.pose.orientation.z = 0.0;
   MoveObs_traj.pose.orientation.w = 1.0;
   MoveObs_traj.color.r = 0.48;
   MoveObs_traj.color.g = 0.4;
   MoveObs_traj.color.b = 0.9;
   MoveObs_traj.color.a = 0.8;
   
   MoveObs_traj.points.clear();
   MoveObs_traj.points.push_back(MoveObs_pt);

}
void MvObsPtUpdate(const ros::TimerEvent& event)
{
    pub_mv_obs_traj.publish(MoveObs_traj);
    
    // MoveObs_pt.x += _mvObs_vel * t_duration;
    MoveObs_pt.y += _mvObs_vel * t_duration;
    // MoveObs_pt.z += ?;
    if(MoveObs_pt.y < _gl_yl || MoveObs_pt.y > -1.0*_gl_yl)
    {
        MoveObs_traj.points.clear();
        MoveObs_pt.y = -1.0*_gl_yl;
    }
    MoveObs_traj.points.push_back(MoveObs_pt);

    geometry_msgs::Pose MoveObs_pose;
    MoveObs_pose.position = MoveObs_pt;
    MoveObs_pose.orientation.x = 0.0;
    MoveObs_pose.orientation.y = _mvObs_vel;
    MoveObs_pose.orientation.z = 0.0;
    MoveObs_pose.orientation.w = 1.0;
    pub_mv_obs.publish(MoveObs_pose);

    // printf("MoveObs_traj.size = %d \n", (int)MoveObs_traj.points.size());
}

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;
  
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // if ((fabs(pt.z - _odom.pose.pose.position.z) / (pt.x - _odom.pose.pose.position.x)) >
      //     tan(M_PI / 12.0))
      //   continue;
      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) > tan(M_PI / 6.0))
        continue; 

      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.34) continue; 

      _local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "map";

  pub_cloud.publish(_local_map_pcd);
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  nh.getParam("map/x_size", _x_size);
  nh.getParam("map/y_size", _y_size);
  nh.getParam("map/z_size", _z_size);
  nh.getParam("map/mv_obs_vel", _mvObs_vel);

  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  // local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  pub_mv_obs = nh.advertise<geometry_msgs::Pose>("MvObs_Info", 10);
  pub_mv_obs_traj = nh.advertise<visualization_msgs::Marker>("MvObs_trajectory", 1);

  double sensing_duration = 1.0 / sensing_rate * 2.5;
  _resolution = 0.2;
  _inv_resolution = 1.0 / _resolution;
  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);
  t_duration = 0.01;
  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  MvObs_update_timer = nh.createTimer(ros::Duration(t_duration), MvObsPtUpdate);

  MoveObstaclesInit();

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
