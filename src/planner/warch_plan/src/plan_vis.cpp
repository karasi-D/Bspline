#include "plan_vis.h"

using namespace Eigen;
using namespace std;
using std::cout;
using std::endl;

namespace warch_plan
{

    PlanVisualization::PlanVisualization(ros::NodeHandle &nh){
        
        node = nh;
        
        nh.param("map/resolution", _resolution, 0.2);
        nh.param("uav_id", uav_id, 0);
        
        _corridor_vis_pub  = nh.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);
        _fm_path_vis_pub   = nh.advertise<visualization_msgs::MarkerArray>("path_vis", 1);
        _grid_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);
        _nodes_vis_pub     = nh.advertise<visualization_msgs::Marker>("expanded_nodes_vis", 1);
        _traj_vis_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_vis", 1);
        _target_vis_pub    = nh.advertise<visualization_msgs::Marker>("target_vis", 1);
        _local_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("vis_map_local", 1);
        _inf_map_vis_pub   = nh.advertise<sensor_msgs::PointCloud2>("vis_map_inflate", 1);
        
    }
    void PlanVisualization::visTargetPoint(Vector3d target)
    {
        visualization_msgs::Marker node_vis; 
        node_vis.header.frame_id = "world";
        node_vis.header.stamp = ros::Time::now();
        node_vis.ns = "b_traj/visited_target";
        node_vis.type = visualization_msgs::Marker::CUBE;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;
        node_vis.color.a = 0.3;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;

        node_vis.scale.x = _resolution;
        node_vis.scale.y = _resolution;
        node_vis.scale.z = _resolution;

        geometry_msgs::Point pt;
        pt.x = target(0);
        pt.y = target(1);
        pt.z = target(2);
        node_vis.points.push_back(pt);

        _target_vis_pub.publish(node_vis);
    }
    
    void PlanVisualization::visPath(vector<Vector3d> &path)
    {
        for(auto & mk: path_vis.markers) 
            mk.action = visualization_msgs::Marker::DELETE;

        _fm_path_vis_pub.publish(path_vis);
        path_vis.markers.clear();

        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "b_traj/fast_marching_path";
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;

        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.a = 0.6;
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;

        int idx = 0;
        for(int i = 0; i < int(path.size()); i++)
        {
            mk.id = idx;

            mk.pose.position.x = path[i](0); 
            mk.pose.position.y = path[i](1); 
            mk.pose.position.z = path[i](2);  

            mk.scale.x = _resolution;
            mk.scale.y = _resolution;
            mk.scale.z = _resolution;

            idx ++;
            path_vis.markers.push_back(mk);
        }

        _fm_path_vis_pub.publish(path_vis);
    }

    void PlanVisualization::visCorridor(vector<Cube> &corridor, bool _is_proj_cube)
    {   
        for(auto & mk: cube_vis.markers) 
            mk.action = visualization_msgs::Marker::DELETE;
        
        _corridor_vis_pub.publish(cube_vis);

        cube_vis.markers.clear();

        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "corridor";
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;

        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.a = 0.1;
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.5;

        int idx = 0;
        for(int i = 0; i < int(corridor.size()); i++)
        {   
            mk.id = idx;

            mk.pose.position.x = (corridor[i].vertex(0, 0) + corridor[i].vertex(3, 0) ) / 2.0; 
            mk.pose.position.y = (corridor[i].vertex(0, 1) + corridor[i].vertex(1, 1) ) / 2.0; 

            if(_is_proj_cube)
                mk.pose.position.z = 0.0; 
            else
                mk.pose.position.z = (corridor[i].vertex(0, 2) + corridor[i].vertex(4, 2) ) / 2.0; 

            mk.scale.x = (corridor[i].vertex(0, 0) - corridor[i].vertex(3, 0) );
            mk.scale.y = (corridor[i].vertex(1, 1) - corridor[i].vertex(0, 1) );

            if(_is_proj_cube)
                mk.scale.z = 0.05; 
            else
                mk.scale.z = (corridor[i].vertex(0, 2) - corridor[i].vertex(4, 2) );

            idx ++;
            cube_vis.markers.push_back(mk);
        }

        _corridor_vis_pub.publish(cube_vis);
    }

    void PlanVisualization::visGridPath( vector<Vector3d> &grid_path)
    {   
        for(auto & mk: grid_vis.markers) 
            mk.action = visualization_msgs::Marker::DELETE;

        _grid_path_vis_pub.publish(grid_vis);
        grid_vis.markers.clear();

        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "b_traj/grid_path";
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;

        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.a = 1.0;
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;

        int idx = 0;
        for(int i = 0; i < int(grid_path.size()); i++)
        {
            mk.id = idx;

            mk.pose.position.x = grid_path[i](0); 
            mk.pose.position.y = grid_path[i](1); 
            mk.pose.position.z = grid_path[i](2);  

            mk.scale.x = _resolution;
            mk.scale.y = _resolution;
            mk.scale.z = _resolution;

            idx ++;
            grid_vis.markers.push_back(mk);
        }

        _grid_path_vis_pub.publish(grid_vis);
    }

    void PlanVisualization::visExpNode( vector<GridNodePtr> &nodes)
    {   
        visualization_msgs::Marker node_vis; 
        node_vis.header.frame_id = "world";
        node_vis.header.stamp = ros::Time::now();
        node_vis.ns = "b_traj/visited_nodes";
        node_vis.type = visualization_msgs::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;
        node_vis.color.a = 0.3;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;

        node_vis.scale.x = _resolution;
        node_vis.scale.y = _resolution;
        node_vis.scale.z = _resolution;

        geometry_msgs::Point pt;
        for(int i = 0; i < int(nodes.size()); i++)
        {
            Vector3d coord = nodes[i]->coord;
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);

            node_vis.points.push_back(pt);
        }

        _nodes_vis_pub.publish(node_vis);
    }

    void PlanVisualization::visBezierTrajectory(MatrixXd &polyCoeff, VectorXd &time, VectorXd _C, double _vis_traj_width)
    {   
        visualization_msgs::Marker traj_vis;
        TrajectoryGenerator _trajectoryGenerator; // only use here

        traj_vis.header.stamp       = ros::Time::now();
        traj_vis.header.frame_id    = "world";

        traj_vis.ns = "trajectory/trajectory";
        traj_vis.id = 0;
        traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        
        traj_vis.action = visualization_msgs::Marker::DELETE;
        // _checkTraj_vis_pub.publish(traj_vis);
        // _stopTraj_vis_pub.publish(traj_vis);

        traj_vis.action = visualization_msgs::Marker::ADD;
        traj_vis.scale.x = _vis_traj_width;
        traj_vis.scale.y = _vis_traj_width;
        traj_vis.scale.z = _vis_traj_width;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;
        traj_vis.color.r = 1.0;
        traj_vis.color.g = 0.0;
        traj_vis.color.b = 0.0;
        traj_vis.color.a = 0.6;

        double traj_len = 0.0;
        int count = 0;
        Vector3d cur, pre;
        cur.setZero();
        pre.setZero();
        
        traj_vis.points.clear();

        Vector3d state;
        geometry_msgs::Point pt;

        int segment_num  = polyCoeff.rows();
        int _traj_order = polyCoeff.cols() / 3 - 1;
        for(int i = 0; i < segment_num; i++ ){
            for (double t = 0.0; t < 1.0; t += 0.05 / time(i), count += 1){
                state = _trajectoryGenerator.getPosFromBezier( polyCoeff, _C, _traj_order, t, i );
                cur(0) = pt.x = time(i) * state(0);
                cur(1) = pt.y = time(i) * state(1);
                cur(2) = pt.z = time(i) * state(2);
                traj_vis.points.push_back(pt);

                if (count) traj_len += (pre - cur).norm();
                pre = cur;
            }
        }

        printf("\033[42;37m[UAV %d - VisTraj]\033[0m The length of the trajectory: %.3lfm. ", uav_id, traj_len);
        _traj_vis_pub.publish(traj_vis);
    }

    void PlanVisualization::VisLocalMap(const sensor_msgs::PointCloud2 & local_map)
    {
        _local_map_vis_pub.publish(local_map);
    }
    
    void PlanVisualization::VisInflateLocalMap(const sensor_msgs::PointCloud2 & inflateMap)
    {
        _inf_map_vis_pub.publish(inflateMap);
    }

    void PlanVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
    {

        if (_traj_vis_pub.getNumSubscribers() == 0)
        {
        return;
        }

        vector<Eigen::Vector3d> list;
        for (int i = 0; i < optimal_pts.cols(); i++)
        {
        Eigen::Vector3d pt = optimal_pts.col(i).transpose();
        list.push_back(pt);
        }
        Eigen::Vector4d color(1, 1, 0, 1);
        displayMarkerList(_traj_vis_pub, list, 0.2, color, id);
    }

    // // real ids used: {id, id+1000}
    void PlanVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                    Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
    {
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = color(0);
        sphere.color.g = line_strip.color.g = color(1);
        sphere.color.b = line_strip.color.b = color(2);
        sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;
        for (int i = 0; i < int(list.size()); i++)
        {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        //if (show_sphere) sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
        }
        //if (show_sphere) pub.publish(sphere);
        pub.publish(line_strip);
    }
}