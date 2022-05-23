#ifndef _PLAN_VIS_H_
#define _PLAN_VIS_H_

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
//#include <Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <algorithm>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include "data_type.h"
#include "trajectory_generator.h"

using namespace Eigen;
using namespace std;

namespace warch_plan
{
  class PlanVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher _corridor_vis_pub, _fm_path_vis_pub, _grid_path_vis_pub;
    ros::Publisher _nodes_vis_pub, _traj_vis_pub, _target_vis_pub;
    ros::Publisher _local_map_vis_pub, _inf_map_vis_pub;
    
    
    visualization_msgs::MarkerArray grid_vis;
    visualization_msgs::MarkerArray cube_vis;
    visualization_msgs::MarkerArray path_vis; 

    double _resolution;
    int uav_id;
  public:
    PlanVisualization(/* args */) {}
    ~PlanVisualization() {}
    PlanVisualization(ros::NodeHandle &nh);

    void visPath(vector<Vector3d> &path);
    void visCorridor(vector<Cube> &corridor, bool _is_proj_cube);
    void visGridPath( vector<Vector3d> &grid_path);
    void visTargetPoint(Vector3d target);
    void visExpNode( vector<GridNodePtr> &nodes);
    void visBezierTrajectory(MatrixXd &polyCoeff, VectorXd &time, VectorXd _C, double _vis_traj_width);
    void VisLocalMap(const sensor_msgs::PointCloud2 & local_map);
    void VisInflateLocalMap(const sensor_msgs::PointCloud2 & local_map);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                    Eigen::Vector4d color, int id,  bool show_sphere = true);
  public:
    typedef std::shared_ptr<PlanVisualization> Ptr;
  };
} // namespace bz_plan
#endif