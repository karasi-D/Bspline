#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;

ros::Publisher _all_map_pub;
ros::Publisher _all_ground_pub;
ros::Publisher _mv_obs_pub;
// ros::Subscriber _odom_sub;

int _obs_num, _cir_num;
double _x_size, _y_size, _z_size, _resolution, _sense_rate;
double _init_x, _init_y, _init_z, _group_distance; // init start point
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;
std::string map_frame_name;

bool _has_odom = false;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 globalGround_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMapPtr(new pcl::PointCloud<pcl::PointXYZ>);

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;  

void MapGenerate()
{    
   random_device rd;
   default_random_engine eng(rd());

   uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h );
   uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h );
   uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
   uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

   uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(_x_l + 1.0, _x_h - 1.0);
   uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(_y_l + 1.0, _y_h - 1.0);
   uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(_w_c_l    , _w_c_h    );

   uniform_real_distribution<double> rand_roll      = uniform_real_distribution<double>(- M_PI,     + M_PI);
   uniform_real_distribution<double> rand_pitch     = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
   uniform_real_distribution<double> rand_yaw       = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
   uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
   uniform_real_distribution<double> rand_num       = uniform_real_distribution<double>(0.0, 1.0); 

   pcl::PointXYZ pt_random;

   // 1. put some circles
   for(int i = 0; i < _cir_num; i ++)
   {
      double x0, y0, z0, R;
      std::vector<Vector3d> circle_set;

      x0   = rand_x_circle(eng);
      y0   = rand_y_circle(eng);
      z0   = rand_h(eng) / 2.0;  
      R    = rand_r_circle(eng);

      // if(sqrt( pow(x0-_init_x, 2) + pow(y0-_init_y, 2) ) < _group_distance ) 
      //    continue;

      double a, b;
      a = rand_ellipse_c(eng);
      b = rand_ellipse_c(eng);

      double x, y, z;
      Vector3d pt3;
      
      for(double theta = -M_PI; theta < M_PI; theta += 0.025)
      {  
         x = a * cos(theta) * R;
         y = b * sin(theta) * R;
         z = 0;
         pt3 << x, y, z;
         circle_set.push_back(pt3);
      }
      // Define a random 3d rotation matrix
      Matrix3d Rot;
      double roll,  pitch, yaw;
      double alpha, beta,  gama;
      roll  = rand_roll(eng); // alpha
      pitch = rand_pitch(eng); // beta
      yaw   = rand_yaw(eng); // gama

      alpha = roll;
      beta  = pitch;
      gama  = yaw;

      double p = rand_num(eng);
      if(p < 0.5)
      {
         beta = M_PI / 2.0;
         gama = M_PI / 2.0;
      }

      Rot << cos(alpha) * cos(gama)  - cos(beta) * sin(alpha) * sin(gama), - cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama),   sin(alpha) * sin(beta),
             cos(gama)  * sin(alpha) + cos(alpha) * cos(beta) * sin(gama),   cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), - cos(alpha) * sin(beta),        
             sin(beta)  * sin(gama),                                         cos(gama) * sin(beta),                                         cos(beta);
      
      Vector3d pt3_rot;
      // for (auto it = circle_set.begin(); it != circle_set.end(); it++)
      for(auto pt: circle_set)
      {          
         pt3_rot = Rot * pt;
         pt_random.x = pt3_rot(0) + x0 + 0.001;
         pt_random.y = pt3_rot(1) + y0 + 0.001;
         pt_random.z = pt3_rot(2) + z0 + 0.001;

         if(pt_random.z >= 0.0)
            cloudMapPtr->points.push_back( pt_random );
      }
   }

   bool is_kdtree_empty = false;
   if(cloudMapPtr->points.size() > 0)
      kdtreeMap.setInputCloud( cloudMapPtr->makeShared() ); 
   else
      is_kdtree_empty = true;

   // 2. put some pilar
   for(int i = 0; i < _obs_num; i ++)
   {
      double x, y, w, h; 
      x    = rand_x(eng);
      y    = rand_y(eng);
      w    = rand_w(eng);

      // if(sqrt( pow(x-_init_x, 2) + pow(y-_init_y, 2) ) < _group_distance ) 
      //    continue;

      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
      pointIdxSearch.clear();
      pointSquaredDistance.clear();
      
      if(is_kdtree_empty == false)
      {
         if ( kdtreeMap.nearestKSearch (searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0 )
         {
            if(sqrt(pointSquaredDistance[0]) < 1.0 )
               continue;
         }
      }

      x = floor(x/_resolution) * _resolution + _resolution / 2.0;
      y = floor(y/_resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w/_resolution);

      for(int r = -widNum/2.0; r < widNum/2.0; r ++ )
         for(int s = -widNum/2.0; s < widNum/2.0; s ++ ){
            h    = rand_h(eng);  
            int heiNum = ceil(h/_resolution);
            for(int t = 0; t < heiNum; t ++ ){
               pt_random.x = x + (r+0.5) * _resolution;
               pt_random.y = y + (s+0.5) * _resolution;
               pt_random.z = (t+0.5) * _resolution;
               cloudMapPtr->points.push_back( pt_random );
            }
         }
   }

   cloudMapPtr->width = cloudMapPtr->points.size();
   cloudMapPtr->height = 1;
   cloudMapPtr->is_dense = true;

   ROS_WARN("Finished generate random map ");
   
   //kdtreeLocalMap.setInputCloud( cloudMapPtr->makeShared() );
   if(!cloudMapPtr->empty()){
    pcl::io::savePCDFileBinary("/home/karasi/my_code/data/simu_map.pcd", *cloudMapPtr);
    ROS_WARN("random map is saved as simu_map.pcd, size = %d", cloudMapPtr->points.size());
   }
   pcl::toROSMsg(*cloudMapPtr, globalMap_pcd);
   globalMap_pcd.header.frame_id = map_frame_name; 
}

void readMapFromFile()
{
    if(pcl::io::loadPCDFile("/home/karasi/my_code/data/simu_map.pcd", *cloudMapPtr) < 0)
    {
        ROS_ERROR("/home/karasi/my_code/data/simu_map.pcd is not find!");
    }
    ROS_WARN("from file simu_map.pcd get %d points.", cloudMapPtr->points.size());
   pcl::toROSMsg(*cloudMapPtr, globalMap_pcd);
   globalMap_pcd.header.frame_id = map_frame_name; 
}

void RandomMapGenerate(bool ground_map_swt)
{
   if (ground_map_swt)
   {
      cloudMapPtr->clear();
      MapGenerate();
   }
   else
   {// 设置地平面
      cloudMapPtr->clear();

      pcl::PointXYZ pt_ground;
      for (double xi = 2*_x_l; xi < 2*_x_h; xi += _resolution)
      {
         for (double yi = 2*_y_l; yi < 2*_y_h; yi += _resolution)
         {
            pt_ground.x = xi;
            pt_ground.y = yi;
            pt_ground.z = -0.2;
            cloudMapPtr->points.push_back(pt_ground);
         }
      }
      cloudMapPtr->width = cloudMapPtr->points.size();
      cloudMapPtr->height = 1;
      cloudMapPtr->is_dense = true;

      pcl::toROSMsg(*cloudMapPtr, globalGround_pcd);
      globalGround_pcd.header.frame_id = map_frame_name;
   }
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
   _has_odom = true;
}

// void pubSensedPoints(bool ground_map_swt)
void pubSensedPoints()
{
   _all_map_pub.publish(globalMap_pcd);
   // if (ground_map_swt)
   // {
   //    _all_ground_pub.publish(globalGround_pcd);
   // }
   // else
   // {
   //    _all_map_pub.publish(globalMap_pcd);
   // }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_complex");
   ros::NodeHandle n("~");

   _all_map_pub    = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);
   _all_ground_pub = n.advertise<sensor_msgs::PointCloud2>("global_ground", 1);
   _mv_obs_pub     = n.advertise<geometry_msgs::Point>("Gen_MvObs_pt", 1);

   // _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

   n.param("init_state_x", _init_x, 0.0);
   n.param("init_state_y", _init_y, 0.0);
   n.param("init_state_y", _init_z, 0.0);

   n.param("group_distance", _group_distance, 0.0);

   n.param("map/x_size", _x_size, 50.0);
   n.param("map/y_size", _y_size, 50.0);
   n.param("map/z_size", _z_size, 5.0);
   n.param("map_frame_name", map_frame_name, std::string("world"));

   n.param("map/obs_num", _obs_num, 300);
   n.param("map/circle_num", _cir_num, 30);
   n.param("map/resolution", _resolution, 0.2);

   n.param("ObstacleShape/lower_rad", _w_l, 0.3);
   n.param("ObstacleShape/upper_rad", _w_h, 0.8);
   n.param("ObstacleShape/lower_hei", _h_l, 3.0);
   n.param("ObstacleShape/upper_hei", _h_h, 7.0);

   n.param("CircleShape/lower_circle_rad", _w_c_l, 0.3);
   n.param("CircleShape/upper_circle_rad", _w_c_h, 0.8);

   n.param("sensing/rate", _sense_rate, 1.0);

   _x_l = -_x_size / 2.0;
   _x_h = +_x_size / 2.0;

   _y_l = -_y_size / 2.0;
   _y_h = +_y_size / 2.0;

   // MapGenerate();
   readMapFromFile();
   ros::Rate loop_rate(_sense_rate);
   // bool ground_map_swt = false;
   
   while (ros::ok())
   {
      // pubSensedPoints(ground_map_swt);
      // ground_map_swt = !ground_map_swt;
      pubSensedPoints();
      ros::spinOnce();
      loop_rate.sleep();
   }
}
