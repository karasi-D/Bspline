#include "plan_manager.h"

using namespace std;
using namespace Eigen;
using namespace sdf_tools;

namespace backward {
backward::SignalHandling sh;
}

namespace warch_plan
{
    void PlanManager::init(ros::NodeHandle &nh){

        _has_odom  = false;
        _has_map   = false;
        _has_target= false;
        has_fusion = false;
        _has_traj = false;
        useBezierCurve = false;
        swarmSyn = false;
        _traj_id = 0;
        _start_time = ros::TIME_MAX;
        _free_cell = COLLISION_CELL(0.0);
        _obst_cell = COLLISION_CELL(1.0);
        nbrEnv.resize(2);
        nbrInfo.resize(2);
        exec_state_ = PLAN_EXEC_STATE::INIT;
        exec_continues_cnt = 0;
        MoveObsPt = VectorXd::Zero(3);
        // 所有参数以launch文件为准，这里只是预设定
        nh.param("map/margin",     _cloud_margin, 0.25);
        nh.param("map/resolution", _resolution, 0.2);
        nh.param("map/x_size",       _x_size, 50.0);
        nh.param("map/y_size",       _y_size, 50.0);
        nh.param("map/z_size",       _z_size, 5.0 );
        nh.param("map/x_local_size", _x_local_size, -20.0);
        nh.param("map/y_local_size", _y_local_size, -20.0);
        nh.param("map/z_local_size", _z_local_size, 5.0 );

        nh.param("planParam/uav_id",            pp_.uav_id, 1);
        nh.param("planParam/max_vel",           pp_.max_vel_,  1.0);
        nh.param("planParam/max_acc",           pp_.max_acc_,  1.0);
        nh.param("planParam/max_jerk",          pp_.max_jerk_, -1.0);
        nh.param("planParam/sensing_horizon",   pp_.planning_horizen_, 8.0);
        nh.param("planParam/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        nh.param("planParam/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("planParam/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
        nh.param("swarm/group_num",         group_num, 1);
        nh.param("swarm/group_distance",    group_distance, 4.0);

        nh.param("planning/init_x",       _init_x,  0.0);
        nh.param("planning/init_y",       _init_y,  0.0);
        nh.param("planning/init_z",       _init_z,  0.0);
        nh.param("planning/target_x",       _target_x,  0.0);
        nh.param("planning/target_y",       _target_y,  0.0);
        nh.param("planning/target_z",       _target_z,  0.0);
        nh.param("planning/max_inflate",   _max_inflate_iter, 100);
        nh.param("planning/step_length",   _step_length,     2);
        nh.param("planning/cube_margin",   _cube_margin,   0.2);
        
        nh.param("planning/check_horizon", _check_horizon, 10.0);
        nh.param("planning/stop_horizon",  _stop_horizon,  5.0);
        nh.param("planning/is_limit_vel",  _is_limit_vel,  false);
        nh.param("planning/is_limit_acc",  _is_limit_acc,  false);
        
        nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
        nh.param("fsm/thresh_replan_meter", replan_thresh_, -1.0);
        nh.param("optimization/min_order",  _minimize_order, 3.0);
        nh.param("optimization/poly_order", _traj_order,    10);
        nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
        nh.param("vis/is_proj_cube",   _is_proj_cube, true);

        nh.param("planning/is_use_fm",     _is_use_fm,  true);
        nh.param("planning/target_type",     target_type,  2.0);

        useSwarm = true;// true        
        if(pp_.uav_id == 0 || group_num <= 1) useSwarm = false;

        visualization_.reset(new PlanVisualization(nh));
        _map_sub  = nh.subscribe( "map",       1, &PlanManager::rcvPointCloudCallBack, this );
        _odom_sub = nh.subscribe( "odometry",  1, &PlanManager::rcvOdometryCallbck, this );
        _pts_sub  = nh.subscribe( "waypoints", 1, &PlanManager::rcvWaypointsCallback, this );
        _MvObs_sub = nh.subscribe( "MvObsInfo", 10, &PlanManager::rcvMoveObstaclesCallback, this );

        //_traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);
        _traj_pub = nh.advertise<traj_utils::Bspline>("bspline", 10); // bspline_pub_
        _checkTraj_vis_pub = nh.advertise<visualization_msgs::Marker>("check_trajectory", 1);
        _stopTraj_vis_pub  = nh.advertise<visualization_msgs::Marker>("stop_trajectory", 1);
        string GloPathName = string("/uav_") + std::to_string(pp_.uav_id) + string("_GloPath");
        self_Path_pub = nh.advertise<warch_plan::GloPath>(GloPathName.c_str(), 10);

        self_EnvInfo_pub_ = nh.advertise<warch_plan::EnvInfo>("planning/envInfo_self_pub", 10);
        self_FlyInfo_pub_ = nh.advertise<warch_plan::FlyInfo>("planning/flyInfo_self_pub", 10);
        nbr_EnvInfo_sub_1 = nh.subscribe("planning/envInfo_from_nbr_1", 100, &PlanManager::rcvEnvInfoCallback_1, this);
        nbr_FlyInfo_sub_1 = nh.subscribe("planning/flyInfo_from_nbr_1", 100, &PlanManager::rcvFlyInfoCallback_1, this);
        nbr_EnvInfo_sub_2 = nh.subscribe("planning/envInfo_from_nbr_2", 100, &PlanManager::rcvEnvInfoCallback_2, this); //, ros::TransportHints().tcpNoDelay()
        nbr_FlyInfo_sub_2 = nh.subscribe("planning/flyInfo_from_nbr_2", 100, &PlanManager::rcvFlyInfoCallback_2, this); //, ros::TransportHints().tcpNoDelay()
    
        exec_timer_ = nh.createTimer(ros::Duration(0.01), &PlanManager::execStateCallback, this);

        // set group reference target for every uav
        if(group_num == 4)
        {   // 4 
            refer_target.resize(4);
            refer_target[0] = Vector3d(-0.5 * group_distance * 1.732, 0.0, 0.0);
            refer_target[1] = Vector3d(0.0, 0.5 * group_distance, 0.0);
            refer_target[2] = Vector3d(0.0, -0.5 * group_distance, 0.0);
            refer_target[3] = Vector3d(0.5 * group_distance * 1.732, 0.0, 0.0);
        }
        
        if(group_num == 3)
        {
            // 3 
            refer_target.resize(3);
            refer_target[0] = Vector3d(-1.0*group_distance, 0.0, 0.0);
            refer_target[1] = Vector3d(0.0, 1.0 * group_distance, 0.0);
            refer_target[2] = Vector3d(0.0, -1.0 * group_distance, 0.0);
        }


        for(int i = 0; i < (int)nbrInfo.size(); i++) {
            nbrInfo[i].live = false; 
        }
        
        _map_origin << -_x_size/2.0, -_y_size/2.0, 0.0; // 地图原点：xy平面的中心点
        _pt_max_x = + _x_size / 2.0;
        _pt_min_x = - _x_size / 2.0;
        _pt_max_y = + _y_size / 2.0;
        _pt_min_y = - _y_size / 2.0;
        _pt_max_z = + _z_size;
        _pt_min_z = 0.0;

        _inv_resolution = 1.0 / _resolution; // 1.0单位内有_inv_resolution个栅格，每个栅格的边长为 _resolution
        _max_x_id = (int)(_x_size * _inv_resolution);
        _max_y_id = (int)(_y_size * _inv_resolution);
        _max_z_id = (int)(_z_size * _inv_resolution);
        _max_local_x_id = (int)(_x_local_size * _inv_resolution);
        _max_local_y_id = (int)(_y_local_size * _inv_resolution);
        _max_local_z_id = (int)(_z_local_size * _inv_resolution);

        Vector3i GLSIZE(_max_x_id, _max_y_id, _max_z_id); // 对 corridor 膨胀范围设置地图
        Vector3i LOSIZE(_max_local_x_id, _max_local_y_id, _max_local_z_id);
        Vector3d _pt_max(_pt_max_x, _pt_max_y, _pt_max_z);
        Vector3d _pt_min(_pt_min_x, _pt_min_y, _pt_min_z);

        int GLXYZ_SIZE = _max_x_id * _max_y_id * _max_z_id;
        data_vec.resize(GLXYZ_SIZE, 0);
        path_finder = new gridPathFinder(GLSIZE, LOSIZE, pp_.uav_id);
        path_finder->initGridNodeMap(_resolution, _map_origin);

        Translation3d origin_translation( _map_origin(0), _map_origin(1), 0.0);
        Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
        Affine3d origin_transform = origin_translation * origin_rotation;
        collision_map = new CollisionMapGrid(origin_transform, "world", _resolution, _x_size, _y_size, _z_size, _free_cell);
        collision_map_local = nullptr;
        // collision_grid = new CollisionGrid(origin_transform, "world", _resolution, _x_size, _y_size, _z_size);
        corridor_manager = new CorridorGenerate(_pt_min, _pt_max, GLSIZE, _resolution, _max_inflate_iter, _step_length);
        corridor_manager->setParam(pp_.uav_id, pp_.planning_horizen_);

        bspline_optimizer_.reset(new BsplineOptimizer);
        bspline_optimizer_->setParam(nh);

        if(_trajectoryGenerator.setParam(3, 12, _minimize_order, _traj_order, group_distance) == -1)
            ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
    }

    void PlanManager::execStateCallback(const ros::TimerEvent &e)
    {
        exec_timer_.stop(); // To avoid blockage

        static int loop_cnt = 0;
        loop_cnt++;
        if (loop_cnt == 100)
        {
            printPlanExecState();
            if (!_has_odom)
                printf("[UAV %d] no odom.\n", pp_.uav_id);
                // cout << "no odom." << endl;
            if (!_has_target)
                printf("[UAV %d] wait for goal.\n", pp_.uav_id);
                // cout << "wait for goal or trigger." << endl;
            loop_cnt = 0;
        }

    switch (exec_state_)
    {
        case INIT:
        {
            cout << "now is init state" << endl;
            if (!_has_odom)
            {
                goto force_return;
                // return;
            }
            changePlanExecState(WAIT_TARGET, "Plan");
            // if(_has_map && pp_.uav_id > 0){
            //     _end_pt << _target_x, _target_y, _target_z;
            //     _has_target = true;
            //     exec_is_emerg  = true;
            //     changePlanExecState(GEN_SWAM_TRAJ, "Plan");
            // }
            // else if(pp_.uav_id == 0){
            //     changePlanExecState(WAIT_TARGET, "Plan");
            // }
            break;
        }

        case WAIT_TARGET:
        {
            if (!_has_target)
                goto force_return;
            // return;
            exec_is_emerg  = true;
            changePlanExecState(GEN_SWAM_TRAJ, "Plan");
            break;
        }

        case GEN_SWAM_TRAJ: // for swarm
        {
            _has_traj  = false;
            _traj_id = 0;
            if (_has_odom && _has_target )
            {
                if (trajPlanning(_start_pt, _end_pt))
                {
                    changePlanExecState(EXEC_TRAJ, "Plan");
                    exec_continues_cnt = 0;
                }
                else
                { 
                    ROS_ERROR("Failed to generate the first trajectory!!!");
                }
            }
        break;
        }

        case REPLAN_TRAJ:
        {
            if(_has_traj && exec_is_emerg)
            {
                _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
                _traj_pub.publish(_traj);
                changePlanExecState(EMERGENCY_STOP, "checkExecTraj");
                
            }
            _has_traj = false;
            // ros::Time time_now = ros::Time::now();
            // double t_cur = (time_now - _start_time).toSec();
            // double t_delta = ros::Duration(0, 100).toSec();
            // t_cur = t_delta + t_cur;
            // Eigen::Vector3d pre_start = _trajectoryGenerator.getTsPosFromBezier(_bezier_coeff, _seg_time, t_cur);

            if (trajPlanning(_start_pt, _end_pt)) // _start_pt
            {
                changePlanExecState(EXEC_TRAJ, "Plan");
                exec_continues_cnt = 0;
            }
            break;
        }

        case EXEC_TRAJ:
        {
            /* determine if need to replan */        
            _has_traj = true;
            exec_is_emerg = false;
            freeDynamicObstacleArea();
            if(exec_continues_cnt == 0)
            {   
                pubSelf_FlyInfo();
                trajPubAndShow();
                // for(int ni = 0; ni < (int)nbrInfo.size(); ni ++)
                // {
                //     if(!nbrInfo[ni].live) goto force_return;
                // }
            }
            // ROS_WARN("_start_pt = (%f,%f,%f), local_target_pt = (%f,%f,%f), _end_pt = (%f,%f,%f) ",
            //         _start_pt(0), _start_pt(1), _start_pt(2), local_target_pt(0), local_target_pt(1), local_target_pt(2),
            //         _end_pt(0), _end_pt(1), _end_pt(2) );

            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - _start_time).toSec();
            t_cur = min(time_duration, t_cur);

            // close to the start or global target
            if ((_start_pt - _end_pt).norm() < _resolution) 
            {
                if (t_cur > time_duration - 1e-2)
                {
                    _has_target = false;
                    if (_has_traj)
                    {
                        _traj_id = 0;
                        _has_traj = false;
                    } 
                    changePlanExecState(WAIT_TARGET, "Plan");
                    goto force_return;
                }
            }
            else if((_start_pt - local_target_pt).norm() < _stop_horizon 
                && ((local_target_pt - _end_pt).norm() > 1e-3))
            {
                changePlanExecState(REPLAN_TRAJ, "NewStage"); 
            }
            else if (checkExecTraj() && !exec_is_emerg) // t_cur > replan_thresh_
            {
                changePlanExecState(REPLAN_TRAJ, "PlanFresh"); 
            }
            else{
                exec_continues_cnt ++;
            }

            break;
        }

        case EMERGENCY_STOP:
        {
            if (exec_is_emerg) // Avoiding repeated calls
            {
                // callEmergencyStop(odom_pos_);
                if(_has_traj && exec_is_emerg)
                {
                    _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
                    _traj_pub.publish(_traj);
                    _has_traj = false;
                }
            }
            // else
            // {
            //     if ((_start_pt - _end_pt).norm() >= 1e-3 && _start_vel.norm() < 0.1)
            //     changePlanExecState(GEN_SWAM_TRAJ, "Plan");
            //     exec_is_emerg = false;
            // }

            break;
        }
    }

    force_return:;
        exec_timer_.start();
    }

    void PlanManager::rcvOdometryCallbck(const nav_msgs::Odometry odom)
    {
        if(!_has_odom)
            ROS_INFO("[UAV %d] get odom", pp_.uav_id);

        _odom = odom;
        _has_odom = true;

        _start_pt(0)  = _odom.pose.pose.position.x;
        _start_pt(1)  = _odom.pose.pose.position.y;
        _start_pt(2)  = _odom.pose.pose.position.z;

        _start_vel(0) = _odom.twist.twist.linear.x;
        _start_vel(1) = _odom.twist.twist.linear.y;
        _start_vel(2) = _odom.twist.twist.linear.z;

        _start_acc(0) = _odom.twist.twist.angular.x;
        _start_acc(1) = _odom.twist.twist.angular.y;
        _start_acc(2) = _odom.twist.twist.angular.z;

        warch_plan::PathData pt; 
        pt.x = _start_pt(0);
        pt.y = _start_pt(1);
        pt.z = _start_pt(2);
        _path.Gpath.push_back(pt);
        self_Path_pub.publish(_path);

        // if not a number return true
        if( std::isnan(_odom.pose.pose.position.x) || std::isnan(_odom.pose.pose.position.y) || std::isnan(_odom.pose.pose.position.z))
            return;

        // static tf::TransformBroadcaster br;
        // tf::Transform transform;
        // transform.setOrigin( tf::Vector3(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z) );
        // transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quadrotor"));
    }

    // get target point  
    void PlanManager::rcvWaypointsCallback(const nav_msgs::Path & wp)
    {
        if(wp.poses[0].pose.position.z < 0.0)
            return;
        
        if(target_type >= 2.0)
        {
            if(wp.poses[0].pose.position.z > _z_size){
                ROS_WARN("please set value z under (0.0, %f]", _z_size);
                return;
            }
            Vector3d targ_;
            targ_ << wp.poses[0].pose.position.x,
                    wp.poses[0].pose.position.y,
                    wp.poses[0].pose.position.z;
            if(pp_.uav_id <= 0) 
                _end_pt = targ_;
            else
                _end_pt = targ_ + refer_target[pp_.uav_id - 1];
            // ROS_INFO("[UAV %d] receive wayPoints: %f, %f, %f", pp_.uav_id, targ_(0), targ_(1), targ_(2));

        }else{
            _end_pt << _target_x, _target_y, _target_z;
        }
        _has_target = true;
        ROS_INFO("[UAV %d] target is: %f, %f, %f", pp_.uav_id, _end_pt(0), _end_pt(1), _end_pt(2));
        visualization_->visTargetPoint(_end_pt);
    }

    void PlanManager::rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
    {
        if(has_fusion == true){
            ROS_WARN("[uav %d] under fusion EnvInfo", pp_.uav_id);
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZ> cloud_local;
        pcl::fromROSMsg(pointcloud_map, cloud_local);
        // if((int)cloud_local.points.size() == 0)
        //     return;
        if(_has_map == false)
            ROS_INFO("[uav %d] get rcvLocalMap size = %d", pp_.uav_id, (int)cloud_local.points.size());
        
        cloud_local.width = cloud_local.points.size();
        cloud_local.height = 1;
        cloud_local.is_dense = true;
        cloud_local.header.frame_id = "world";
        sensor_msgs::PointCloud2 local_map;
        pcl::toROSMsg(cloud_local, local_map);
        visualization_->VisLocalMap(local_map);

        pcl::PointCloud<pcl::PointXYZ> fusionMap;
        pcl::PointCloud<pcl::PointXYZ> interactAddMap;
        for (int idx = 0; idx < (int)cloud_local.points.size(); idx++){
            auto mk = cloud_local.points[idx];
            pcl::PointXYZ pt(mk.x, mk.y, mk.z);
            fusionMap.push_back(pt);
            int data_idx = GetDataIndex(mk.x, mk.y, mk.z);
                if(data_vec[data_idx] == 1)
                    continue;
            data_vec[data_idx] = 1;
            interactAddMap.push_back(pt);
        }
        _has_map = true;
        
        if((int)interactAddMap.points.size() != 0)
            envInfoFusion(fusionMap, interactAddMap);
        
        geneGridMap(fusionMap); // self_localMap, fusionMap

    }

    void PlanManager::pubSelf_EnvInfo(const pcl::PointCloud<pcl::PointXYZ> & interactAddMap, int update)
    {
        warch_plan::EnvInfo self_env;
        self_env.resolution = _resolution;
        self_env.uav_id_from = this->pp_.uav_id;
        self_env.detect_R = this->_check_horizon;
        self_env.fresh = update;
        // update map
        for (int idx = 0; idx < (int)interactAddMap.points.size(); idx++){
            auto mk = interactAddMap.points[idx];
            warch_plan::OccupyData oc;
            oc.x = mk.x;
            oc.y = mk.y;
            oc.z = mk.z;
            oc.occupy = 1;
            self_env.map.push_back(oc);
        }
        self_EnvInfo_pub_.publish(self_env);
        // ROS_WARN("UAV%d update self_EnvInfo_pub_, envMap = %d, pubMap = %d",pp_.uav_id, (int)interactAddMap.points.size(), (int)self_env.map.size() );
        
    }

    void PlanManager::envInfoFusion(pcl::PointCloud<pcl::PointXYZ> & fusionMap, pcl::PointCloud<pcl::PointXYZ> & interactAddMap){
        if(_has_map == false || useSwarm == false){
            if(_has_map == false)
                ROS_WARN("[UAV %d] not sensed itself local map", pp_.uav_id);
            return;
        }
        return;
        has_fusion = true;

        // ROS_INFO("[UAV %d] nbr envInfo numbers = %d", pp_.uav_id, nbrEnv.size());
        int updateFresh = 1;
        pubSelf_EnvInfo(interactAddMap, updateFresh);
        
        // for syn timetemp
        if(!swarmSyn) ros::Duration(0.5).sleep();
        for(int ki = 0; !swarmSyn&& ki < (int)nbrEnv.size(); ki ++)
        {
            if((int)nbrEnv[ki].map.size() <= 0) 
                ros::Duration(0.25).sleep();
        }
        if(!swarmSyn) swarmSyn = true;

        // begin fusion process 
        // ros::Time time_bef_fusion = ros::Time::now();
        int time_nums = 0;
        
        for(int k = 0; k < (int)group_num; k ++) // (int)nbrEnv.size()
        {
            // ROS_WARN("begin fusion map, %dth", k);
            // interactAddMap.points.clear();
            updateFresh = -1;
            for(int ki = 0; ki < (int)nbrEnv.size(); ki ++)
            {
                if(nbrEnv[ki].fresh != 1) continue;
                for(int i = 0; i < (int)nbrEnv[ki].map.size(); i ++){
                    const warch_plan::OccupyData &oc = nbrEnv[ki].map[i];
                    int data_idx = GetDataIndex(oc.x, oc.y, oc.z);
                    int8_t occupy = max(data_vec[data_idx], oc.occupy);

                    if(occupy > 0 && data_vec[data_idx] <= 0){
                        if(updateFresh != 1) updateFresh = 1;
                        pcl::PointXYZ pt(oc.x, oc.y, oc.z);
                        fusionMap.push_back(pt);
                        interactAddMap.push_back(pt);
                        data_vec[data_idx] = 1;
                    }
                }
                pubSelf_EnvInfo(interactAddMap, updateFresh);
            }
            if(updateFresh == 1) {
                ros::Duration(0.5).sleep();
                time_nums++;
            }            
        }
        
        // ros::Time time_aft_fusion = ros::Time::now();
        // int consist_num = 0;
        // for(int ci = 0; ci < (int)data_vec.size(); ci ++){
        //     if(data_vec[ci] > 0) consist_num++;
        // }
        // double consist = 2 * (double)consist_num / (double)data_vec.size();
        // printf("[UAV %d in fusionMap] -TCost is\033[32m %f \033[0m, -EnvConsist is\033[33m %f \033[0m.\n", pp_.uav_id, (time_aft_fusion - time_bef_fusion).toSec() - 0.5*time_nums, consist);

        has_fusion = false;
    }

    void PlanManager::geneGridMap(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
        
        // ROS_WARN("try to delete last map: %d", collision_map_local != nullptr);
        if(collision_map_local != nullptr)
            delete collision_map_local;
        //ros::Time time_1 = ros::Time::now();
        // collision_map->RestMap();
        
        // (int)((_start_pt(0) - _x_local_size/2.0)  * _inv_resolution + 0.5) 向上取整操作
        double local_c_x = (int)((_start_pt(0) - _x_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;
        double local_c_y = (int)((_start_pt(1) - _y_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;
        double local_c_z = (int)((_start_pt(2) - _z_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;

        _local_origin << local_c_x, local_c_y, local_c_z;

        Translation3d origin_local_translation( _local_origin(0), _local_origin(1), _local_origin(2));
        Quaterniond origin_local_rotation(1.0, 0.0, 0.0, 0.0);
        Affine3d origin_local_transform = origin_local_translation * origin_local_rotation;
        double _buffer_size = 2 * pp_.max_vel_;
        double _x_buffer_size = _x_local_size + _buffer_size;
        double _y_buffer_size = _y_local_size + _buffer_size;
        double _z_buffer_size = _z_local_size + _buffer_size;

        collision_map_local = new CollisionMapGrid(origin_local_transform, "world", _resolution, _x_buffer_size, _y_buffer_size, _z_buffer_size, _free_cell);

        vector<pcl::PointXYZ> inflatePts(20);
        // cloud_inflation.points.clear();
        for (int idx = 0; idx < (int)cloud_in.points.size(); idx++)
        {
            auto mk = cloud_in.points[idx];
            pcl::PointXYZ pt(mk.x, mk.y, mk.z);
            Vector3d mkPt(mk.x, mk.y, mk.z);

            // if( fabs(pt.x - _start_pt(0)) > _x_local_size / 2.0 || fabs(pt.y - _start_pt(1)) > _y_local_size / 2.0 || fabs(pt.z - _start_pt(2)) > _z_local_size / 2.0 )
            //     continue;

            // cloud_local.push_back(pt);
            inflatePts = pointInflate(pt, 1);
            for(int i = 0; i < (int)inflatePts.size(); i++)
            {
                pcl::PointXYZ inf_pt = inflatePts[i];
                Vector3d addPt(inf_pt.x, inf_pt.y, inf_pt.z);
                collision_map_local->Set3d(addPt, _obst_cell);
                collision_map->Set3d(addPt, _obst_cell);
                // collision_grid->setObs(addPt, 1);
                if((mkPt - addPt).norm() < 0.0001) continue;
                cloud_inflation.push_back(inf_pt);
            }
        }
        /*
        // setDynamicObstacleArea();
        {
            for(int i = 0; i < (int)MoveObsArea.size(); i++)
            {
                Vector3d addPt(MoveObsArea[i].x, MoveObsArea[i].y, MoveObsArea[i].z);     

                Vector3i pt_idx = collision_map->LocationToGridIndex(addPt);

                if( collision_map->Get( (int64_t)pt_idx(0), (int64_t)pt_idx(1), (int64_t)pt_idx(2) ).first.occupancy > 0.5 )
                {
                    pcl::PointXYZ pt(0, 0, 0);
                    MoveObsArea[i] = pt;
                }else{
                    collision_map_local->Set3d(addPt, _obst_cell);
                    collision_map->Set3d(addPt, _obst_cell);
                    cloud_inflation.push_back(MoveObsArea[i]);
                }
            }
        }
        */

        cloud_inflation.width = cloud_inflation.points.size();
        cloud_inflation.height = 1;
        cloud_inflation.is_dense = true;
        cloud_inflation.header.frame_id = "world";

        sensor_msgs::PointCloud2 inflateMap; //, localMap;
        pcl::toROSMsg(cloud_inflation, inflateMap);

        visualization_->VisInflateLocalMap(inflateMap);
        //ros::Time time_3 = ros::Time::now();
        //ROS_WARN("Time in receving the map is %f", (time_3 - time_1).toSec());

        if( checkExecTraj() == true && exec_state_ == EXEC_TRAJ )
            changePlanExecState(REPLAN_TRAJ, "EnvFresh");
    }

    vector<pcl::PointXYZ> PlanManager::pointInflate( pcl::PointXYZ pt, int multi)
    {
        // 与实际点云障碍物间保持安全距离 _cloud_margin
        int num   = int(_cloud_margin * _inv_resolution) * multi;
        int num_z = max(1, num / 2);
        vector<pcl::PointXYZ> infPts(20);
        pcl::PointXYZ pt_inf;

        for(int x = -num ; x <= num; x ++ )
            for(int y = -num ; y <= num; y ++ )
                for(int z = -num_z ; z <= num_z; z ++ )
                {
                    pt_inf.x = pt.x + x * _resolution;
                    pt_inf.y = pt.y + y * _resolution;
                    pt_inf.z = pt.z + z * _resolution;

                    infPts.push_back( pt_inf );
                }

        return infPts;
    }

    bool PlanManager::checkExecTraj()
    {
        if( _has_traj == false )
            return false;

        visualization_msgs::Marker _check_traj_vis, _stop_traj_vis;

        geometry_msgs::Point pt;
        _stop_traj_vis.header.stamp    = _check_traj_vis.header.stamp    = ros::Time::now();
        _stop_traj_vis.header.frame_id = _check_traj_vis.header.frame_id = "world";

        _check_traj_vis.ns = "trajectory/check_trajectory";
        _stop_traj_vis.ns  = "trajectory/stop_trajectory";

        _stop_traj_vis.id     = _check_traj_vis.id = 0;
        _stop_traj_vis.type   = _check_traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        _stop_traj_vis.action = _check_traj_vis.action = visualization_msgs::Marker::ADD;

        _stop_traj_vis.scale.x = 2.0 * _vis_traj_width;
        _stop_traj_vis.scale.y = 2.0 * _vis_traj_width;
        _stop_traj_vis.scale.z = 2.0 * _vis_traj_width;

        _check_traj_vis.scale.x = 1.5 * _vis_traj_width;
        _check_traj_vis.scale.y = 1.5 * _vis_traj_width;
        _check_traj_vis.scale.z = 1.5 * _vis_traj_width;

        _check_traj_vis.pose.orientation.x = 0.0;
        _check_traj_vis.pose.orientation.y = 0.0;
        _check_traj_vis.pose.orientation.z = 0.0;
        _check_traj_vis.pose.orientation.w = 1.0;

        _stop_traj_vis.pose = _check_traj_vis.pose;

        _stop_traj_vis.color.r = 0.0;
        _stop_traj_vis.color.g = 1.0;
        _stop_traj_vis.color.b = 0.0;
        _stop_traj_vis.color.a = 1.0;

        _check_traj_vis.color.r = 0.0;
        _check_traj_vis.color.g = 0.0;
        _check_traj_vis.color.b = 1.0;
        _check_traj_vis.color.a = 1.0;

        Vector3d traj_pt;
        double t_s = max(0.0, (_odom.header.stamp - _start_time).toSec());

        for(double t_d = t_s; t_d < _check_horizon; t_d += 0.01)
        {
            traj_pt = local_data_.position_traj_.evaluateDeBoorT(t_d);
            pt.x = traj_pt(0);
            pt.y = traj_pt(1);
            pt.z = traj_pt(2);
            _check_traj_vis.points.push_back(pt);

            if( t_d <= _stop_horizon )
                _stop_traj_vis.points.push_back(pt);

            if( checkCoordObs(traj_pt, MoveObsPt, t_d))
            {
                printf("\033[33m[UAV %d] predicted collision time is %f ahead!\033[0m  ", pp_.uav_id, t_d);

                if( t_d <= _stop_horizon )
                {
                    ROS_ERROR("[UAV %d] emergency occurs in time is %f ahead", pp_.uav_id, t_d);
                    exec_is_emerg = true;
                    // changePlanExecState(EMERGENCY_STOP, "checkExecTraj");
                    // if(exec_state_ == EXEC_TRAJ)
                        changePlanExecState(REPLAN_TRAJ, "checkExecTraj");

                }

                _checkTraj_vis_pub.publish(_check_traj_vis);
                _stopTraj_vis_pub.publish(_stop_traj_vis);
                
                Eigen::MatrixXd control_points(3, 6);
                for (int i = 0; i < 6; i++)
                {
                    control_points.col(i) = traj_pt;
                }

                updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());
                trajPubAndShow();
                return true;
            }
        }
    
    if(useBezierCurve)
    {
        int idx;
        for (idx = 0; idx < _seg_num; ++idx)
        {
            if( t_s  > _seg_time(idx) && idx + 1 < _seg_num)
                t_s -= _seg_time(idx);
            else
                break;
        }

        double duration = 0.0;
        double t_ss;
        for(int i = idx; i < _seg_num; i++ )
        {
            t_ss = (i == idx) ? t_s : 0.0;
            for(double t = t_ss; t < _seg_time(i); t += 0.01)
            {
                double t_d = duration + t - t_ss;
                if( t_d > _check_horizon ) break;
                
                traj_pt = _trajectoryGenerator.getPosFromBezier( _bezier_coeff, t/_seg_time(i), i );
                pt.x = traj_pt(0) = _seg_time(i) * traj_pt(0);
                pt.y = traj_pt(1) = _seg_time(i) * traj_pt(1);
                pt.z = traj_pt(2) = _seg_time(i) * traj_pt(2);
                
                _check_traj_vis.points.push_back(pt);

                if( t_d <= _stop_horizon )
                    _stop_traj_vis.points.push_back(pt);

                if( checkCoordObs(traj_pt, MoveObsPt, t_d))
                {
                    printf("\033[33m[UAV %d] predicted collision time is %f ahead!\033[0m  ", pp_.uav_id, t_d);

                    if( t_d <= _stop_horizon )
                    {
                        ROS_ERROR("[UAV %d] emergency occurs in time is %f ahead", pp_.uav_id, t_d);
                        exec_is_emerg = true;
                        // changePlanExecState(EMERGENCY_STOP, "checkExecTraj");
                        changePlanExecState(REPLAN_TRAJ, "checkExecTraj");

                    }

                    _checkTraj_vis_pub.publish(_check_traj_vis);
                    _stopTraj_vis_pub.publish(_stop_traj_vis);

                    return true;
                }
            }
            duration += _seg_time(i) - t_ss;
        }
    }
        
        _checkTraj_vis_pub.publish(_check_traj_vis);
        _stopTraj_vis_pub.publish(_stop_traj_vis);
        
        // ROS_WARN("checkExecTraj = false!" );
        return false;
    }

    bool PlanManager::checkCoordObs(Vector3d checkPt, Vector3d PreObsPt, double t_d)
    {
        Vector3d dyObsPt = PreObsPt - t_d * MoveObsVel - _start_pt;
        if((dyObsPt - _start_pt).norm() < 0.5)
            // mvdiff(0)) < 2*_resolution && abs(mvdiff(1)) < 2*_resolution && abs(mvdiff(2)) < 2*_resolution) // 
        {
            // preDynamicObstacleCollision(dyObsPt, t_d);
            return true;
        }
        if(collision_map->Get(checkPt(0), checkPt(1), checkPt(2)).first.occupancy > 0.5 )
            return true;
        return false;
    }
    
    double PlanManager::velMapping(double d, double max_v)
    {
        double vel;

        if( d <= 0.25)
            vel = 2.0 * d * d;
        else if(d > 0.25 && d <= 0.75)
            vel = 1.5 * d - 0.25;
        else if(d > 0.75 && d <= 1.0)
            vel = - 2.0 * (d - 1.0) * (d - 1.0) + 1;
        else
            vel = 1.0;

        return vel * max_v;
    }

    bool PlanManager::path_searching(bool swarmFlag, Vector3d plan_start_pt, Vector3d plan_end_pt, vector<Cube>& corridor, MatrixXd& local_path, VectorXd& seg_time)
    {
      if(_is_use_fm)
      {
            printf("\033[43;37m[UAV %d Plan_node]\033[0m  use 'Fast Marching Method': ", pp_.uav_id);
            // use fmm find path
            ros::Time time_1 = ros::Time::now();
            float oob_value = INFINITY;
            auto EDT = collision_map_local->ExtractDistanceField(oob_value);
            ros::Time time_2 = ros::Time::now();
            printf("\033[32m[UAV %d - EDT]\033[0m time in generate EDT is %f; ", pp_.uav_id, (time_2 - time_1).toSec());

            unsigned int idx = 0;
            double max_vel = pp_.max_vel_ * 0.75;
            vector<unsigned int> obs;
            Vector3d pt;
            vector<int64_t> pt_idx;
            double flow_vel = 0.0;

            unsigned int size_x = (unsigned int)(_max_x_id);
            unsigned int size_y = (unsigned int)(_max_y_id);
            unsigned int size_z = (unsigned int)(_max_z_id);

            Coord3D dimsize {size_x, size_y, size_z};
            FMGrid3D grid_fmm(dimsize); // internal is vector<FMCell> cells_;

            // 计算速度场 grid_fmm
            for(unsigned int k = 0; k < size_z; k++)
            {
                for(unsigned int j = 0; j < size_y; j++)
                {
                    for(unsigned int i = 0; i < size_x; i++)
                    {
                        idx = k * size_y * size_x + j * size_x + i;
                        pt << (i + 0.5) * _resolution + _map_origin(0),
                            (j + 0.5) * _resolution + _map_origin(1),
                            (k + 0.5) * _resolution + _map_origin(2);

                        Vector3i index = collision_map_local->LocationToGridIndex(pt);

                        if(collision_map_local->Inside(index))
                        {
                            double d = sqrt(EDT.GetImmutable(index).first.distance_square) * _resolution;
                            flow_vel = velMapping(d, max_vel);
                        }
                        else
                            flow_vel = max_vel;

                        if( k == 0 || k == (size_z - 1) || j == 0 || j == (size_y - 1) || i == 0 || i == (size_x - 1) )
                            flow_vel = 0.0;

                        grid_fmm[idx].setVelocity(flow_vel);
                        if (grid_fmm[idx].isOccupied())
                            obs.push_back(idx);
                    }
                }
            }
            grid_fmm.setOccupiedCells(std::move(obs));
            grid_fmm.setLeafSize(_resolution);

            Vector3d startIdx3d = (plan_start_pt - _map_origin) * _inv_resolution;
            Vector3d endIdx3d   = (plan_end_pt   - _map_origin) * _inv_resolution;
            Coord3D goal_point = {(unsigned int)startIdx3d[0], (unsigned int)startIdx3d[1], (unsigned int)startIdx3d[2]};
            Coord3D init_point = {(unsigned int)endIdx3d[0],   (unsigned int)endIdx3d[1],   (unsigned int)endIdx3d[2]};

            unsigned int startIdx, goalIdx;
            vector<unsigned int> startIndices;
            grid_fmm.coord2idx(init_point, startIdx);
            grid_fmm.coord2idx(goal_point, goalIdx);
            startIndices.push_back(startIdx);
            grid_fmm[goalIdx].setVelocity(max_vel); // setOccupancy
            Solver<FMGrid3D>* fm_solver = new FMMStar<FMGrid3D>("FMM*_Dist", TIME); // LSM, FMM

            fm_solver->setEnvironment(&grid_fmm);
            fm_solver->setInitialAndGoalPoints(startIndices, goalIdx);
            
            vector<double> nbr_diff(size_x * size_y * size_z, 0.0);
            if(swarmFlag){
                setNbrPathDiff(nbr_diff, grid_fmm, size_x, size_y, size_z);
            }
            fm_solver->setNbrPath(std::move(nbr_diff));

            // 从终点向起点反向寻找一条最短路径，compute()
            ros::Time time_bef_fm = ros::Time::now();
            if(fm_solver->compute(max_vel) == -1)
            {
                printf("\033[41;37m[UAV %d - Fast Marching Node]\033[0m No path can be found; ", pp_.uav_id);
                changePlanExecState(REPLAN_TRAJ, "trajPlanning"); // _traj_pub：ACTION_WARN_IMPOSSIBLE
                return false;
            }
            ros::Time time_aft_fm = ros::Time::now();
            printf("\033[32m[UAV %d - FMM]\033[0m Time in Fast Marching computing is %f; ", pp_.uav_id, (time_aft_fm - time_bef_fm).toSec() );

            Path3D path3D;
            vector<double> path_vels, time;
            GradientDescent< FMGrid3D > grad3D;
            grid_fmm.coord2idx(goal_point, goalIdx);

            // find a path by gradient descent
            /// input: grid_fmm, goalIdx:从源点 goalIdx 开始findPath
            /// output: path3D, path_vels, time
            /// return: success 1, false -1
            if(grad3D.gradient_descent(grid_fmm, goalIdx, path3D, path_vels, time) == -1)
            {
                printf("\033[41;37m[UAV %d - Fast Marching Node]\033[0m FMM failed, valid path not exists; ", pp_.uav_id);
                changePlanExecState(REPLAN_TRAJ, "trajPlanning"); // _traj_pub：ACTION_WARN_IMPOSSIBLE
                return false;
            }

            // 将 path3D 按照可视化地图规整化为 path_coord, 并将其可视化
            vector<Vector3d> path_local, path_global;
            path_local.push_back(plan_start_pt);
            path_global.push_back(plan_start_pt);
            double coord_x, coord_y, coord_z;
            bool find_mid = false;
            for( int i = 0; i < (int)path3D.size(); i++)
            {
                coord_x = max(min( (path3D[i][0]+0.5) * _resolution + _map_origin(0), _x_size), -_x_size);
                coord_y = max(min( (path3D[i][1]+0.5) * _resolution + _map_origin(1), _y_size), -_y_size);
                coord_z = max(min( (path3D[i][2]+0.5) * _resolution, _z_size), 0.0);

                Vector3d pt(coord_x, coord_y, coord_z);
                // only choose local traj opt
                if(!find_mid && (i > (int)path3D.size() / 2) && ((pt - plan_start_pt).norm() > pp_.planning_horizen_))  // 
                    find_mid = true;
                if(!find_mid) path_local.push_back(pt);
                
                path_global.push_back(pt);

            }
            local_target_pt = path_local.back();
            if((local_target_pt - plan_end_pt).norm() < 1.0){
                local_target_pt = plan_end_pt;
                path_local.back() = plan_end_pt;
            }
            my_path_coord = path_local;

            visualization_->visPath(path_global);
            // 简化路径(sortPath) + 生成安全区-飞行走廊，即生成cube并膨胀处理
            ros::Time time_bef_corridor = ros::Time::now();
            sortPath(path_local, time, local_target_pt);
            corridor = corridor_manager->corridorGeneration(path_local, time, collision_map); // collision_map 对应GLSIZE, collision_map_local对应LOSIZE
            ros::Time time_aft_corridor = ros::Time::now();
            printf("\033[32m[UAV %d - corridor]\033[0m Time consume in corridor generation is %f;  ", pp_.uav_id, (time_aft_corridor - time_bef_corridor).toSec());

            timeAllocation(corridor, time, local_path, seg_time);
            visualization_->visCorridor(corridor, _is_proj_cube);
            delete fm_solver;
      }
      else{
        
        printf("\033[43;37m[UAV %d - Plan_node]\033[0m  use 'E-A star Method': ", pp_.uav_id);
        // use astar find path
        path_finder->linkLocalMap(collision_map_local, _local_origin);
        path_finder->AstarSearch(plan_start_pt, plan_end_pt);
        // path_finder->AstarSearch(plan_end_pt, plan_start_pt);
        vector<Vector3d> path_global = path_finder->getPath();
        vector<GridNodePtr> searchedNodes = path_finder->getVisitedNodes();
        path_finder->resetLocalMap();
        // 将 path3D 按照可视化地图规整化为 path_local, 并将其可视化
        vector<Vector3d> path_local;
        for( int i = 0; i < (int)path_global.size(); i++)
        {
            Vector3d pt = path_global[i];
            path_local.push_back(pt);
            // only choose local traj opt
            if((i > (int)path_global.size() / 2) && ((pt - plan_start_pt).norm() > pp_.planning_horizen_))  // 
                break; // find local_target_pt
        }
        local_target_pt = path_local.back();
        if((local_target_pt - plan_end_pt).norm() < 1.0){
            local_target_pt = plan_end_pt;
            path_local.back() = plan_end_pt;
        }
        my_path_coord = path_local;

        visualization_->visGridPath(path_local);
        visualization_->visExpNode(searchedNodes);

        ros::Time time_bef_corridor = ros::Time::now();
        corridor = corridor_manager->corridorGeneration(path_local, collision_map);
        ros::Time time_aft_corridor = ros::Time::now();
        printf("\033[32m[UAV %d - corridor]\033[0m Time consume in corridor generation is %f;  ", pp_.uav_id, (time_aft_corridor - time_bef_corridor).toSec());

        timeAllocation(corridor, local_path, seg_time);
        visualization_->visCorridor(corridor, _is_proj_cube);
      }
        double localpath_len = 0.0;
        for( int i = 1; i < (int)local_path.cols(); i++)
        {
            Vector3d pt1 = Vector3d(local_path(0,i-1), local_path(1,i-1),local_path(2,i-1));
            Vector3d pt2 = Vector3d(local_path(0,i), local_path(1,i),local_path(2,i));
            localpath_len += (pt2 - pt1).norm();
        }
        printf("\033[32m[UAV %d - TrajSeg]\033[0m The trajectory segment = %d, localpath_len = %.3lf.\n", pp_.uav_id, (int)seg_time.rows(), localpath_len);
        return true;
    }
    
    bool PlanManager::localTraj_opt(bool swarmFlag, Vector3d opt_start_pt, Vector3d opt_end_pt, vector<Cube>& corridor)
    {
        MatrixXd pos = MatrixXd::Zero(2,3);
        MatrixXd vel = MatrixXd::Zero(2,3);
        MatrixXd acc = MatrixXd::Zero(2,3);
        
        pos.row(0) = opt_start_pt;
        pos.row(1) = opt_end_pt; // plan_end_pt;
        vel.row(0) = _start_vel;
        acc.row(0) = _start_acc;

        // 后端：轨迹优化，基于BezierPolyCoeff
        double obj;
        ros::Time time_bef_opt = ros::Time::now();

        printf("\033[44;37m[UAV %d - TrajInfo]\033[0m: swarmFlag = %d, traj_id = %d; ", pp_.uav_id, swarmFlag, _traj_id);
        vector<double> highOrderMaxVal= {pp_.max_vel_, pp_.max_acc_};
        if(_trajectoryGenerator.BezierPloyCoeffGeneration
            (swarmFlag , nbrInfo, corridor, pos, vel, acc, highOrderMaxVal, _cube_margin, _is_limit_vel, 
            _is_limit_acc, obj, _bezier_coeff ) == -1 )
        {
            printf("\033[31m[UAV %d] Cannot find a feasible and optimal solution, somthing wrong with the mosek solver.\033[0m  ", pp_.uav_id);
            changePlanExecState(REPLAN_TRAJ, "trajPlanning"); // _traj_pub：ACTION_WARN_IMPOSSIBLE
            return false;
        }
        else
        {
            ros::Time time_aft_opt = ros::Time::now();
            printf("\033[32m[UAV %d - JCost]\033[0m The objective of the program is %f; ", pp_.uav_id, obj);
            printf("\033[32m[UAV %d - TCost]\033[0m The time consumation of the program is %f.\n ", pp_.uav_id, (time_aft_opt - time_bef_opt).toSec());
            
            _seg_num = corridor.size();
            _seg_time.resize(_seg_num);
            for(int i = 0; i < _seg_num; i++)
                _seg_time(i) = corridor[i].t;
            time_duration = _seg_time.sum();
            _start_time = _odom.header.stamp;
            _traj_id ++;
            return true;

        }
    }

    bool PlanManager::trajPlanning(Vector3d plan_start_pt, Vector3d plan_end_pt)
    {
        if( _has_target == false || _has_odom == false) // || _has_map == false 
        {
            printf("\033[41;37m[UAV%d state]\033[0m: _has_odom = %d, _has_target= %d, _has_map = %d, has_fusion = %d", pp_.uav_id, _has_odom, _has_target,_has_map,has_fusion);
            changePlanExecState(WAIT_TARGET, "trajPlanning");
            return false;
        }
        
        // get nbr info
        bool swarmFlag = useSwarm;
        int nbrs = 0;
        for(int i = 0; i < (int)nbrInfo.size(); i++)
            if( nbrInfo[i].live) nbrs ++; 
        if( nbrs == 0 ) {
            swarmFlag = false;
            ROS_INFO("no live nbr, then swarmFlag = false.");
        }

        bool plan_success = false;
        vector<Cube> corridor;
        MatrixXd local_path;
        VectorXd seg_time;
        plan_success = path_searching(swarmFlag, plan_start_pt, plan_end_pt, corridor, local_path, seg_time);
        
        if(plan_success){
            // vector<Vector3d> gl_traj;
            ROS_WARN("path_searching success, begin traj opt");
            if(useBezierCurve)
                plan_success = localTraj_opt(swarmFlag, plan_start_pt, local_target_pt, corridor);
            else
                plan_success = getPoly2Ctrl_pts(local_path, seg_time);
        }
        ros::Duration(10.0).sleep();
        return plan_success;
    }

    bool PlanManager::getPoly2Ctrl_pts(MatrixXd& local_path, VectorXd& seg_time)
    {
        Vector3d local_target_vel = Eigen::Vector3d::Zero();
        bspline_optimizer_->setLocalTargetPt(local_target_pt);

        ros::Time t_start = ros::Time::now();
        // ros::Duration t_init, t_opt, t_refine;
        
        /*** STEP 1: INIT : get point_set, start_end_derivatives ***/
        double ts = (_start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        vector<Eigen::Vector3d> point_set, start_end_derivatives;
        static bool flag_first_call = true ;
        bool flag_regenerate = false;
        do
        {
        point_set.clear();
        start_end_derivatives.clear();
        flag_regenerate = false;

        if (flag_first_call /*|| ( _start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
        {
            flag_first_call = false;
            ROS_WARN("flag_first_call");
            double dist = (_start_pt - local_target_pt).norm();
            double evlu_time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
            double time = max(evlu_time, seg_time.sum());
            /** STEP 1.1:  generate a minimum-jerk piecewise monomial polynomial-based trajectory  **/
            PolynomialTraj gl_traj;
            if((int)local_path.size() > 2)
                gl_traj = PolynomialTraj::minSnapTraj(local_path, _start_vel, local_target_vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), seg_time);
            else if((int)local_path.size() == 2)
                gl_traj = PolynomialTraj::one_segment_traj_gen(_start_pt, _start_vel, Eigen::Vector3d::Zero(), local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
            else 
                {return false;}

            double t;
            bool flag_too_far;
            ts *= 1.5; // ts will be divided by 1.5 in the next
            do
            {
            ts /= 1.5;
            point_set.clear();
            flag_too_far = false;
            Eigen::Vector3d last_pt = gl_traj.evaluate(0);
            for (t = 0; t < time; t += ts)
            {
                Eigen::Vector3d pt = gl_traj.evaluate(t);
                if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
                {
                    flag_too_far = true;
                    break;
                }
                last_pt = pt;
                point_set.push_back(pt);
            }
            } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
            
            t -= ts;
            start_end_derivatives.push_back(gl_traj.evaluateVel(0));
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
            start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
        }
        else // Initial path generated from previous trajectory.
        {
            ROS_WARN("generate point_set from local_data_");
            double t;
            double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

            vector<double> pseudo_arc_length; 
            vector<Eigen::Vector3d> segment_point;// 离散全局轨迹点化为segment_point，pseudo_arc_length为走到当前点的路径总长度
            pseudo_arc_length.push_back(0.0);
            for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
            {
                segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                if (t > t_cur)
                {
                    pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                }
            }
            t -= ts;

            double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
            if (poly_time > ts)
            {
            PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                            local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                            local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                            local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

            for (t = ts; t < poly_time; t += ts)
            {
                if (!pseudo_arc_length.empty())
                {
                segment_point.push_back(gl_traj.evaluate(t));
                pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                }
                else
                {
                ROS_ERROR("pseudo_arc_length is empty, return!");
                continous_failures_count_++;
                return false;
                }
            }
            }

            double sample_length = 0;
            double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
            size_t id = 0;
            do
            {
            cps_dist /= 1.5;
            point_set.clear();
            sample_length = 0;
            id = 0;
            while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
            {
                if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                {
                point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                    (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                sample_length += cps_dist;
                }
                else
                id++;
            }
            point_set.push_back(local_target_pt);
            } while (point_set.size() < 7); // If the start point is very close to end point, this will help

            start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
            start_end_derivatives.push_back(Eigen::Vector3d::Zero());

            if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
            {
            flag_regenerate = true;
            }
        }
        } while (flag_regenerate);

        ROS_WARN("got point_set.size = %d, back = (%f,%f,%f) ", (int)point_set.size(), point_set.back()(0), point_set.back()(1), point_set.back()(2));
        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
        ROS_WARN("got ctrl_pts.size = %d  x %d", (int)ctrl_pts.rows(), (int)ctrl_pts.cols());

        Eigen::MatrixXd ctrl_pts_temp;
        ControlPoints cpsOfTraj;
        cpsOfTraj.points = ctrl_pts;
        cpsOfTraj.size = (int)ctrl_pts.cols();
        double final_cost = 1e8;
        bool bspline_opt_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, cpsOfTraj, ts);
        ROS_WARN("bspline_opt_success = %d", bspline_opt_success);
        if(!bspline_opt_success) 
        {
            continous_failures_count_ ++;
            return false;
        }
        UniformBspline pos = UniformBspline(ctrl_pts_temp, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
        updateTrajInfo(pos, ros::Time::now());

        static int count_success = 0;
        count_success++;
        _seg_num = _seg_time.size();
        time_duration = _seg_time.sum();
        _start_time = _odom.header.stamp;
        _traj_id ++;

        // static double sum_time = 0;
        // sum_time += (t_init + t_opt + t_refine).toSec();
        //cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

        // success. YoY
        continous_failures_count_ = 0;
        return true;
    }

    void PlanManager::timeAllocation(vector<Cube> & corridor, MatrixXd& local_path, VectorXd& seg_time)
    {
        vector<Vector3d> points;
        points.push_back (_start_pt);

        for(int i = 1; i < (int)corridor.size(); i++)
            points.push_back(corridor[i].center);

        points.push_back(local_target_pt);//  _end_pt

        local_path = MatrixXd::Zero(3, (int)points.size());
        local_path(0,0) = _start_pt(0);
        local_path(1,0) = _start_pt(1);
        local_path(2,0) = _start_pt(2);
        seg_time = VectorXd::Zero((int)points.size() - 1);
   
        double _Vel = pp_.max_vel_ * 0.6;
        double _Acc = pp_.max_acc_ * 0.6;

        for (int k = 0; k < (int)points.size() - 1; k++)
        {
            double dtxyz;
            Vector3d p0   = points[k];
            Vector3d p1   = points[k + 1];
            Vector3d d    = p1 - p0;
            Vector3d v0(0.0, 0.0, 0.0);
            local_path(0,k+1) = p1(0);
            local_path(1,k+1) = p1(1);
            local_path(2,k+1) = p1(2);

            if( k == 0) v0 = _start_vel;

            double D    = d.norm();
            double V0   = v0.dot(d / D);
            double aV0  = fabs(V0);

            double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1);
            double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
            double dcct = _Vel / _Acc;
            double dccd = _Acc * dcct * dcct / 2;

            if (D < aV0 * aV0 / (2 * _Acc))
            {
                double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
                double t2 = aV0 / _Acc;
                dtxyz     = t1 + t2;
            }
            else if (D < accd + dccd)
            {
                double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
                double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
                double t3 = (aV0 + _Acc * t2) / _Acc;
                dtxyz     = t1 + t2 + t3;
            }
            else
            {
                double t1 = acct;
                double t2 = (D - accd - dccd) / _Vel;
                double t3 = dcct;
                dtxyz     = t1 + t2 + t3;
            }
            corridor[k].t = dtxyz;
            seg_time(k) = dtxyz;
        }
    }

    void PlanManager::timeAllocation(vector<Cube> & corridor, vector<double>& time, MatrixXd& local_path, VectorXd& seg_time)
    {
        seg_time = VectorXd::Zero((int)corridor.size());

        for(int i  = 0; i < (int)corridor.size() - 1; i++)
        {
            double duration  = (corridor[i].t - corridor[i+1].t);
            seg_time(i) = duration;
        }
        seg_time((int)corridor.size() - 1) = corridor.back().t;

        vector<Vector3d> points;
        points.push_back (_start_pt);
        for(int i = 1; i < (int)corridor.size(); i++)
            points.push_back(corridor[i].center);
        points.push_back (local_target_pt);

        local_path = MatrixXd::Zero(3, (int)points.size());
        local_path(0,0) = _start_pt(0);
        local_path(1,0) = _start_pt(1);
        local_path(2,0) = _start_pt(2);

        double _Vel = pp_.max_vel_ * 0.6;
        double _Acc = pp_.max_acc_ * 0.6;

        Eigen::Vector3d initv = _start_vel;
        for(int i = 0; i < (int)points.size() - 1; i++)
        {
            double dtxyz;

            Eigen::Vector3d p0   = points[i];
            Eigen::Vector3d p1   = points[i + 1];
            Eigen::Vector3d d    = p1 - p0;
            Eigen::Vector3d v0(0.0, 0.0, 0.0);
            local_path(0,i+1) = p1(0);
            local_path(1,i+1) = p1(1);
            local_path(2,i+1) = p1(2);

            if( i == 0) v0 = initv;

            // std: d.norm()= sqrt(d^2)；eigen: 点乘A.dot(B) = A·B，叉乘A.cross(B) = A * B
            double D    = d.norm();
            double V0   = v0.dot(d / D);
            double aV0  = fabs(V0);

            double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1);
            double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
            double dcct = _Vel / _Acc;
            double dccd = _Acc * dcct * dcct / 2;
            // 讨论两段间距 与 加速减速的对应情况，全程以最大加速度飞行
            if (D < aV0 * aV0 / (2 * _Acc)) // 从aV0直接减速至零的距离
            {
                double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
                double t2 = aV0 / _Acc;
                dtxyz     = t1 + t2;
            }
            else if (D < accd + dccd) // 从V0加速至最大，立即减速至0的距离
            {
                double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
                double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
                double t3 = (aV0 + _Acc * t2) / _Acc;
                dtxyz     = t1 + t2 + t3;
            }
            else // 从V0加速至最大，匀速飞行一段，然后减速至0
            {
                double t1 = acct;
                double t2 = (D - accd - dccd) / _Vel;
                double t3 = dcct;
                dtxyz     = t1 + t2 + t3;
            }

            if(dtxyz < seg_time(i) * 0.5) //  dtxyz < seg_time(i) * 0.5
                seg_time(i) = dtxyz; // if FM given time in this segment is rediculous long, use the new value
        }

        for(int i = 0; i < (int)corridor.size(); i++)
        {
            corridor[i].t = seg_time(i);
        }

    }
    
    void PlanManager::sortPath(vector<Vector3d> & path_coord, vector<double> & time, Vector3d plan_end_pt)
    {
        vector<Vector3d> path_tmp;
        vector<double> time_tmp;

        for (int i = 0; i < (int)path_coord.size(); i += 1)
        {
            if( i )
                if( std::isinf(time[i]) || time[i] == 0.0 || time[i] == time[i-1] )
                    continue;

            if( (path_coord[i] - plan_end_pt).norm() < 0.2)
                break;

            path_tmp.push_back(path_coord[i]);
            time_tmp.push_back(time[i]);
        }
        path_coord = path_tmp;
        time       = time_tmp;
    }

    void PlanManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
    {
        local_data_.start_time_ = time_now;
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }
    
    void PlanManager::trajPubAndShow()
    {
        if(useBezierCurve)
        {
            _traj = _trajectoryGenerator.getBezierTraj(_bezier_coeff, _seg_time,  _seg_num,  _traj_id, _start_time );
            _traj_pub.publish(_traj);
            visualization_->visBezierTrajectory(_bezier_coeff, _seg_time, _trajectoryGenerator.getC(), _vis_traj_width);
            return;
        }
        
        auto& info = local_data_;
        traj_utils::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info.start_time_;
        bspline.traj_id = info.traj_id_;

        Eigen::MatrixXd pos_pts = info.position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info.position_traj_.getKnot();
        // cout << knots.transpose() << endl;
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        /* 1. publish traj to traj_server */
        _traj_pub.publish(bspline); //bspline_pub_
        
        /* 2. publish traj to the next drone of swarm */

        /* 3. publish traj for visualization */
        visualization_->displayOptimalList(info.position_traj_.get_control_points(), 0);
    }
    
    void PlanManager::pubSelf_FlyInfo()
    {
        warch_plan::FlyInfo self_fly;
        self_fly.uav_id_from = this->pp_.uav_id;
        // self_fly.odom = this->_odom;
        self_fly.all_time = time_duration;
        self_fly.issue = _traj_id;
        self_fly.traj_order = _traj_order;
        if(_traj_id == 0 || pp_.uav_id == 0 || exec_state_ == EMERGENCY_STOP) self_fly.issue = -1;
        if(self_fly.issue != -1) self_fly.t_duration = (_odom.header.stamp - _start_time).toSec();
        self_fly.traj_start_t = _start_time.toSec();
        self_fly.pos.x = _start_pt(0);
        self_fly.pos.y = _start_pt(1);
        self_fly.pos.z = _start_pt(2);
        self_fly.vel.x = _start_vel(0);
        self_fly.vel.y = _start_vel(1);
        self_fly.vel.z = _start_vel(2);
        self_fly.acc.x = _start_acc(0);
        self_fly.acc.y = _start_acc(1);
        self_fly.acc.z = _start_acc(2);

        int n_ploy = _traj_order+1;
        for(int i = 0; i < _seg_num; i++ )
        {   for(int p = 0; p < 3; p ++){
                for(int j = 0; j < n_ploy; j++){
                    double cof = _bezier_coeff(i, p * n_ploy + j);
                    self_fly.coeff.push_back(cof);
                }
            }
            self_fly.seg_t.push_back(_seg_time(i));
        }

        for(int i = 0; i < (int)my_path_coord.size(); i ++){
            warch_plan::PathData pt; 
            pt.x = my_path_coord[i](0);
            pt.y = my_path_coord[i](1);
            pt.z = my_path_coord[i](2);
            self_fly.path_coord.push_back(pt);
        }
        self_FlyInfo_pub_.publish(self_fly);
    }

    void PlanManager::updateNbrFlyInfo(const warch_plan::FlyInfo & NbriFlyInfo, int id)
    {
        int nbr_id = NbriFlyInfo.uav_id_from;
        
        // ROS_INFO("[Fusion] [UAV %d] receive nbr [uav%d] envinfo", uav_id, NbriFlyInfo.uav_id_from);

        bool nbrCheckCollision = false;
        // if() nbrCheckCollision = true;
        Nbr_Info &info_ = nbrInfo[id];
        info_.id = nbr_id;
        info_.live = true;
        info_.t_duration = 0.0;
        if(NbriFlyInfo.issue == -1) 
        {
            info_.live = false;
            return;
        }
        
        info_.t_duration = NbriFlyInfo.t_duration;
        // pos
        info_.pos(0)  = NbriFlyInfo.pos.x;
        info_.pos(1)  = NbriFlyInfo.pos.y;
        info_.pos(2)  = NbriFlyInfo.pos.z;
        // vel
        info_.vel(0) = NbriFlyInfo.vel.x;
        info_.vel(1) = NbriFlyInfo.vel.y;
        info_.vel(2) = NbriFlyInfo.vel.z;
        // acc
        // info_.Acc(0) = NbriFlyInfo.acc.x;
        // info_.Acc(1) = NbriFlyInfo.acc.y;
        // info_.Acc(2) = NbriFlyInfo.acc.z;
        
        // outof connection distance
        if((_start_pt - info_.pos).norm() > group_distance * 4.0f / 3.0f){
            info_.live = false; 
            return;
        }
        
        if(nbrInfo[id].issue != NbriFlyInfo.issue)
        {
            nbrInfo[id].issue = NbriFlyInfo.issue;
            info_.centerDiff = refer_target[pp_.uav_id-1] - refer_target[nbr_id-1];
            // update nbr coeff and seg_t
            int n_ploy = NbriFlyInfo.traj_order + 1;
            int nbr_seg = (int)NbriFlyInfo.seg_t.size();
            
            info_.coeff = MatrixXd::Zero(nbr_seg, 3*n_ploy );
            info_.seg_t = VectorXd::Zero(nbr_seg);
            for(int i = 0, var_shift = 0; i < nbr_seg; i++ )
            {
                for(int j = 0; j < 3 * n_ploy; j++)
                    info_.coeff(i , j) = NbriFlyInfo.coeff[j + var_shift];
                var_shift += 3 * n_ploy;
                info_.seg_t(i) = NbriFlyInfo.seg_t[i];
            }
            // update nbr path_coord
            for(int i = 0; i < (int)NbriFlyInfo.path_coord.size(); i ++)
            {
                warch_plan::PathData pt = NbriFlyInfo.path_coord[i];
                info_.path_coord.push_back(Vector3d(pt.x, pt.y, pt.z));
            }
        }
        
        // /* Check Collision */
        if (exec_state_ == EXEC_TRAJ)
        {
            for(int ni = 0; ni < (int)nbrInfo.size(); ni ++)
            {
                if(!nbrInfo[ni].live) return;
            }
            double my_traj_t_duration = (_odom.header.stamp - _start_time).toSec();
            double nbr_traj_t_duration = NbriFlyInfo.t_duration;
            double nbr_ts = abs(my_traj_t_duration - nbr_traj_t_duration);
            double t_head = max(my_traj_t_duration, nbr_traj_t_duration);

            double my_traj_start_t = _start_time.toSec();
            double nbr_traj_start_t = NbriFlyInfo.traj_start_t;
            double t_tail = min(my_traj_start_t + time_duration / 3, nbr_traj_start_t + NbriFlyInfo.all_time);
            // if(nbr_ts > 2*replan_thresh_) {
            //     nbrCheckCollision = true;
            // }
            /*
            for (double t = t_head; t < t_tail && info_.live; t += 0.03)
            {
                if(nbrCheckCollision)  break;
                double nbr_dis = 0.0;
                if(useBezierCurve)
                nbr_dis = (_trajectoryGenerator.getTsPosFromBezier(_bezier_coeff, _seg_time, t - my_traj_start_t) -
                    _trajectoryGenerator.getTsPosFromBezier(info_.coeff, info_.seg_t, t - nbr_traj_start_t)).norm();
                else  
                    nbr_dis = (local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_t) - info_.local_data_.position_traj_.evaluateDeBoorT(t - nbr_traj_start_t).norm());
                if (nbr_dis < 0.5 || nbr_dis > group_distance)
                {
                    // safe distance 0.5m, group distance Lg
                    nbrCheckCollision = true;
                    break;
                }
            }
            */
            if (nbrCheckCollision)
                changePlanExecState(REPLAN_TRAJ, "SWARM_TRAJ_CHECK");
        }

        
    }

    void PlanManager::setNbrPathDiff(std::vector<double> & nbr_diff, FMGrid3D & grid_fmm, unsigned int size_x, unsigned int size_y, unsigned int size_z)
    {
        for(int i = 0; i < (int)nbrInfo.size(); i ++){
            if(!nbrInfo[i].live) continue;

            queue<unsigned int> q_path;
            vector<bool> Cell_vis(size_x * size_y * size_z, false);
            for(int j = 0; j < (int)nbrInfo[i].path_coord.size(); j ++){
                Vector3d ptIdx3d = (nbrInfo[i].path_coord[j] - refer_target[nbrInfo[i].id - 1] + refer_target[pp_.uav_id - 1] - _map_origin) * _inv_resolution;
                unsigned int nbr_idx = (unsigned int)ptIdx3d[2] * size_y * size_x + (unsigned int)ptIdx3d[1] * size_x + (unsigned int)ptIdx3d[0];
                Cell_vis[nbr_idx] = true;
                // nbr_diff[nbr_idx] = 0.0; 
                nbr_diff[nbr_idx] = group_distance;
                q_path.push(nbr_idx);
            }
            int depth = double(2.0) /double(_resolution) ;
            while(q_path.size() && depth){
                int q_len = q_path.size();
                depth --;
                for(int qi = 0; qi < q_len; qi ++){
                    int q_idx = q_path.front();
                    q_path.pop();

                    for(int dz = -1; dz < 2; dz ++){
                        for(int dy = -1; dy < 2; dy ++){
                            for(int dx = -1; dx < 2; dx ++){
                                if(!dx && !dy && !dz) continue;
                                unsigned int d_idx = q_idx + dz * size_y * size_x + dy * size_x + dx;
                                if((d_idx >= size_x * size_y * size_z) || grid_fmm.getCell(d_idx).isOccupied() || Cell_vis[d_idx]) continue;
                                double d_dis = dx + dy + dz - (3 - sqrt(3) * min(dx, min(dy, dz)) 
                                    - (2 - sqrt(2)) *(dx + dy + dz - min(dx, min(dy, dz)) - max(dx, max(dy, dz))) - min(dx, min(dy, dz)));
                                //double d_dis = sqrt(dx*dx + dy*dy + dz*dz);
                                // nbr_diff[d_idx] = max(nbr_diff[d_idx], nbr_diff[q_idx] + d_dis*_resolution);
                                nbr_diff[d_idx] = min(nbr_diff[d_idx], nbr_diff[q_idx] - d_dis*_resolution);
                                Cell_vis[d_idx] = true;
                                q_path.push(d_idx);
                            }
                        }
                    }
                }
            }// complete nbr_diff
        }
    }

    void PlanManager::preDynamicObstacleCollision(Vector3d checkPt, double t_s)
    {
        MoveObsArea.clear();
        int num   = int(_cloud_margin * _inv_resolution);
        Vector3d pre_num = t_s * MoveObsVel;
        int dx = num + (int) pre_num(0), dy = num + (int) pre_num(1), dz = num/2 + (int) pre_num(2);
        pcl::PointXYZ pt_inf;

        for(int x = -dx ; x <= dx; x ++ )
            for(int y = -dy ; y <= dy; y ++ )
                for(int z = -dz ; z <= dz; z ++ )
                {
                    pt_inf.x = checkPt(0) + x * _resolution;
                    pt_inf.y = checkPt(1) + y * _resolution;
                    pt_inf.z = checkPt(2) + z * _resolution;

                    MoveObsArea.push_back( pt_inf );
                }

    }

    void PlanManager::setDynamicObstacleArea()
    {
        for(int i = 0; i < (int)MoveObsArea.size(); i++)
        {
            Vector3d addPt(MoveObsArea[i].x, MoveObsArea[i].y, MoveObsArea[i].z);     

            Vector3i pt_idx = collision_map->LocationToGridIndex(addPt);

            if( collision_map->Get( (int64_t)pt_idx(0), (int64_t)pt_idx(1), (int64_t)pt_idx(2) ).first.occupancy > 0.5 )
            {
                pcl::PointXYZ pt(0, 0, 0);
                MoveObsArea[i] = pt;
            }else{
                collision_map_local->Set3d(addPt, _obst_cell);
                collision_map->Set3d(addPt, _obst_cell);
            }
        }
    }
    
    void PlanManager::freeDynamicObstacleArea()
    {
        for(int i = 0; i < (int)MoveObsArea.size(); i++)
        {
            Vector3d clearPt(MoveObsArea[i].x, MoveObsArea[i].y, MoveObsArea[i].z);            

            if(clearPt(0) < 0.0001 && clearPt(1) < 0.0001 && clearPt(2) < 0.0001)
                continue;
            collision_map->Set3d(clearPt, _free_cell); 
        }
        MoveObsArea.clear();
    }

}