#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include "mosek.h"
#include "bezier_base.h"
#include "data_type.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

using namespace std;
using namespace Eigen;

class TrajectoryGenerator {
private:
        // MatrixXd _bezier_coeff;
        int traj_order;
        double minimize_order;
        double group_distance; // multi uav distance
        // bezier basis constant
        MatrixXd MQM, M, _FM, MGM, MVM;
        VectorXd _C, _Cv, _Ca, _Cj;

public:
        TrajectoryGenerator(){}
        ~TrajectoryGenerator(){}
        int setParam(int poly_order_min, int poly_order_max, double min_order, int _traj_order, double _group_distance);
        VectorXd getC()  { return _C; }
        VectorXd getC_v(){ return _Cv; }
        VectorXd getC_a(){ return _Ca; }
        VectorXd getC_j(){ return _Cj; }
        /* Use Bezier curve for the trajectory */ // const int nbrs,
       int BezierPloyCoeffGeneration(
            bool isSwarm,
            const std::vector<Nbr_Info> & nbrInfo,
             vector<Cube> &corridor,
            const MatrixXd &pos,
            const MatrixXd &vel,
            const MatrixXd &acc,
            const vector<double> & highOrderMaxVal,
            const double margin,
            const bool & isLimitVel,
            const bool & isLimitAcc,
            double & obj, 
            MatrixXd & PolyCoeff); 

        quadrotor_msgs::PolynomialTrajectory getBezierTraj(const MatrixXd & polyCoeff, const VectorXd & _seg_time, int _seg_num, int _traj_id, ros::Time t_now );
        Vector3d getPosFromBezier(const MatrixXd & polyCoeff, const VectorXd & _C,  int _traj_order, double t_now, int seg_now );
        Vector3d getPosFromBezier(const MatrixXd & polyCoeff,  double t_now, int seg_now );
        Vector3d getVelFromBezier(const MatrixXd & polyCoeff,  double t_now, int seg_now);
        VectorXd getStateFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now );
        Vector3d getTsPosFromBezier(const MatrixXd & polyCoeff, const VectorXd & seg_time, double t_s);        
        int getNbrSegFromTime(const VectorXd & seg_time, double t_s);
        
};

#endif