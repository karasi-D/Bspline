#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

#include "trajectory_generator.h"
#include "data_type.h"
#include "fmm_utils.h"
#include "a_star.h"
#include "plan_vis.h"
#include "plan_manager.h"
#include "backward.hpp"

#include <warch_plan/FlyInfo.h>
#include <warch_plan/EnvInfo.h>
#include <warch_plan/OccupyData.h>
#include <warch_plan/GloPath.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <traj_utils/DataDisp.h>

using namespace Eigen;
using namespace std;
using namespace sdf_tools;
using namespace planner;

namespace warch_plan
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  struct PlanParameters
  {
    /* planning algorithm parameters */
    int uav_id; // single drone: uav_id <= 0, swarm: uav_id >= 1
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    bool use_distinctive_trajs;

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  class PlanManager
  {
    // SECTION stable
  public:
    PlanManager(){}
    ~PlanManager(){
      delete corridor_manager;
      delete path_finder;
    }

    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    PlanParameters pp_;

    // use B spline
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    BsplineOptimizer::Ptr bspline_optimizer_;
    
    bool useBezierCurve;
    quadrotor_msgs::PolynomialTrajectory _traj;
    TrajectoryGenerator _trajectoryGenerator;

    void init(ros::NodeHandle &nh);
    void execStateCallback(const ros::TimerEvent &e);

    void rcvWaypointsCallback(const nav_msgs::Path & wp);
    void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
    void rcvOdometryCallbck(const nav_msgs::Odometry odom);
    inline void rcvMoveObstaclesCallback(const geometry_msgs::Pose mv_obs)
    { 
      MoveObsPt(0) = mv_obs.position.x; 
      MoveObsPt(1) = mv_obs.position.y; 
      MoveObsPt(2) = mv_obs.position.z; 
      MoveObsVel(0) = mv_obs.orientation.x; 
      MoveObsVel(1) = mv_obs.orientation.y; 
      MoveObsVel(2) = mv_obs.orientation.z; 
      if((_start_pt - MoveObsPt).norm() < 3.0){
        if( checkExecTraj() == true && exec_state_ == EXEC_TRAJ )
            changePlanExecState(REPLAN_TRAJ, "DynamicObsFresh");
      }
    }
    void geneGridMap(const pcl::PointCloud<pcl::PointXYZ>& cloud_in);
    void envInfoFusion(pcl::PointCloud<pcl::PointXYZ> & fusionMap);
    vector<pcl::PointXYZ> pointInflate( pcl::PointXYZ pt, int multi);
    void sortPath(vector<Vector3d> & path_coord, vector<double> & time, Vector3d plan_end_pt);
    void timeAllocation(vector<Cube> & corridor, MatrixXd& local_path, VectorXd& seg_time);
    void timeAllocation(vector<Cube> & corridor, vector<double>& time, MatrixXd& local_path, VectorXd& seg_time);

    bool trajPlanning(Vector3d plan_start_pt, Vector3d plan_end_pt);
    bool path_searching(bool swarmFlag, Vector3d plan_start_pt, Vector3d plan_end_pt, vector<Cube>& corridor, MatrixXd& global_path, VectorXd& seg_time);
    bool localTraj_opt(bool swarmFlag, Vector3d opt_start_pt, Vector3d opt_end_pt, vector<Cube>& corridor);
    bool getPoly2Ctrl_pts(MatrixXd& global_path, VectorXd& seg_time);
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);
    void trajPubAndShow();
    
    double velMapping(double d, double max_v);
    bool checkExecTraj();
    bool checkCoordObs(Vector3d checkPt, Vector3d PreObsPt, double t_d);
    void setDynamicObstacleArea();
    void freeDynamicObstacleArea();
    void preDynamicObstacleCollision(Vector3d checkPt, double t_s);

    // ros related
    PlanVisualization::Ptr visualization_;
    ros::Subscriber _map_sub, _pts_sub, _odom_sub, _ngbInfo_sub;
    ros::Publisher _traj_pub, _selfInfo_pub, self_Path_pub; 
    ros::Publisher _local_map_vis_pub, _inf_map_vis_pub, _checkTraj_vis_pub, _stopTraj_vis_pub;
    
  private:
    COLLISION_CELL _free_cell, _obst_cell;
    nav_msgs::Odometry _odom;
    warch_plan::GloPath _path;
    ros::Time _start_time;
    ros::Timer exec_timer_;
    Vector3d MoveObsPt, MoveObsVel;
    vector<pcl::PointXYZ> MoveObsArea;
    // simulation param from launch file
    double _vis_traj_width, _resolution, _inv_resolution;
    double _cloud_margin, _cube_margin, _check_horizon, _stop_horizon;
    double no_replan_thresh_, replan_thresh_, time_duration;
    double _x_size, _y_size, _z_size, _x_local_size, _y_local_size, _z_local_size;    
    bool   _is_use_fm, _is_proj_cube, _is_limit_vel, _is_limit_acc;
    int    _step_length, _max_inflate_iter, _traj_order, exec_continues_cnt;
    double _minimize_order, target_type;

    // useful global variables
    bool _has_odom, _has_map, _has_target, _has_traj, exec_is_emerg, useSwarm;
    
    Vector3d _start_pt, _start_vel, _start_acc, _end_pt, local_target_pt;
    double _init_x, _init_y, _init_z, _target_x, _target_y, _target_z;
    Vector3d _map_origin, _local_origin;
    double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
    int _max_x_id, _max_y_id, _max_z_id, _max_local_x_id, _max_local_y_id, _max_local_z_id;

    // trajectory related
    int _traj_id, _seg_num;
    VectorXd _seg_time;
    MatrixXd _bezier_coeff;

    // useful object
    pcl::PointCloud<pcl::PointXYZ> cloud_inflation;
    CollisionMapGrid *collision_map_local;
    CollisionMapGrid *collision_map;
    
    // CollisionGrid *collision_grid;
    CorridorGenerate *corridor_manager;
    gridPathFinder *path_finder;
    
    // group request info
    int group_num;
    bool has_fusion, swarmSyn, useFusion;
    double group_distance;
    std::vector<Vector3d> refer_target;
    std::vector<int8_t> data_vec;
    std::vector<Vector3d> my_path_coord;
    std::vector<warch_plan::EnvInfo> nbrEnv;
    std::vector<Nbr_Info> nbrInfo;
    // multi uav here
    ros::Publisher  self_EnvInfo_pub_, self_FlyInfo_pub_;
    ros::Subscriber nbr_EnvInfo_sub_1, nbr_EnvInfo_sub_2, nbr_FlyInfo_sub_1, nbr_FlyInfo_sub_2;
    ros::Subscriber _MvObs_sub;

    void rcvEnvInfoCallback_1(const warch_plan::EnvInfo & NbrEnv1)
    {
        nbrEnv[0] = NbrEnv1; 
    }
    void rcvEnvInfoCallback_2(const warch_plan::EnvInfo & NbrEnv2)
    {
        nbrEnv[1] = NbrEnv2; 
    }
    void pubSelf_EnvInfo(const pcl::PointCloud<pcl::PointXYZ> & interactAddMap, int update);
    void envInfoFusion(pcl::PointCloud<pcl::PointXYZ> & fusionMap, pcl::PointCloud<pcl::PointXYZ> & interactAddMap);
    
    void rcvFlyInfoCallback_1(const warch_plan::FlyInfo & NbrFly){updateNbrFlyInfo(NbrFly, 0); }
    void rcvFlyInfoCallback_2(const warch_plan::FlyInfo & NbrFly){updateNbrFlyInfo(NbrFly, 1); }
    void pubSelf_FlyInfo();
    void updateNbrFlyInfo(const warch_plan::FlyInfo & NbriFlyInfo, int id);

    void setNbrPathDiff(std::vector<double> & nbr_diff, FMGrid3D & grid_fmm, unsigned int size_x, unsigned int size_y, unsigned int size_z);
    inline int GetDataIndex(const double coord_x, const double coord_y, const double coord_z) {
      if (coord_x < _pt_min_x || coord_y < _pt_min_y || coord_z < _pt_min_z ||
          coord_x >= _pt_max_x || coord_y >= _pt_max_y || coord_z >= _pt_max_z)
        return 0;

      int x_index = static_cast<int>((coord_x - _pt_min_x) * _inv_resolution);
      int y_index = static_cast<int>((coord_y - _pt_min_y) * _inv_resolution);
      int z_index = static_cast<int>((coord_z - _pt_min_z) * _inv_resolution);
      if (x_index >= 0 && y_index >= 0 && z_index >= 0 && x_index < _max_x_id && y_index < _max_y_id && z_index < _max_z_id)
      {
            return (x_index * (_max_y_id * _max_z_id)) + (y_index * _max_z_id) + z_index;
      }
      else return -1;
    }
    
  public:
    typedef unique_ptr<PlanManager> Ptr;
  private:
    enum PLAN_EXEC_STATE
    {
      INIT = 0,
      WAIT_TARGET,
      GEN_SWAM_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    }exec_state_;
    int continous_failures_count_{0};
    void printPlanExecState()
    {
      static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_SWAM_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
      printf("[UAV %d -state] is %s.\n", pp_.uav_id, state_str[int(exec_state_)].c_str());
    }
    
    void changePlanExecState(PLAN_EXEC_STATE new_state, string pos_call)
    {
      static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_SWAM_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
      int pre_s = int(exec_state_);
      exec_state_ = new_state;
      printf("\033[36m[UAV %d: %s-Change]: from %s to %s\033[0m.\n  ", pp_.uav_id, pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str() );
    }
    // !SECTION
  };
} // namespace warch_plan

#endif