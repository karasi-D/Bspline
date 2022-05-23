#include "trajectory_generator.h"
using namespace std;
using namespace Eigen;

// static void MSKAPI printstr(void *handle, MSKCONST char str[])
// {
//   printf("%s",str);
// }

int TrajectoryGenerator::setParam(int poly_order_min, int poly_order_max, double min_order, int _traj_order, double _group_distance)
{
        Bernstein _bernstein;
        if(_bernstein.setParam(poly_order_min, poly_order_max, min_order) == -1)
            return -1;

        MQM = _bernstein.getMQM()[_traj_order];
        MGM = _bernstein.getMGM()[_traj_order];
        MVM = _bernstein.getMVM()[_traj_order];
        M   = _bernstein.getM()[_traj_order];
        _FM  = _bernstein.getFM()[_traj_order];
        _C   = _bernstein.getC()[_traj_order];
        _Cv  = _bernstein.getC_v()[_traj_order];
        _Ca  = _bernstein.getC_a()[_traj_order];
        _Cj  = _bernstein.getC_j()[_traj_order];

        traj_order =  _traj_order;
        minimize_order = min_order;
        group_distance = _group_distance;
        return 0;
}

int TrajectoryGenerator::BezierPloyCoeffGeneration(
            bool isSwarm,
            const std::vector<Nbr_Info> & nbrInfo,
            vector<Cube> &corridor,
            const MatrixXd &pos, const MatrixXd &vel, const MatrixXd &acc,
            const vector<double> & highOrderMaxVal,
            const double margin,
            const bool & isLimitVel, const bool & isLimitAcc,
            double & obj,
            MatrixXd & PolyCoeff)  // define the order to which we minimize.   1 -- velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap
{
#define ENFORCE_VEL  isLimitVel // whether or not adding extra constraints for ensuring the velocity feasibility
#define ENFORCE_ACC  isLimitAcc // whether or not adding extra constraints for ensuring the acceleration feasibility
#define USE_SWARM    isSwarm

    // traj_order = traj_order;
    double initScale = corridor.front().t;
    double lstScale  = corridor.back().t;
    int segment_num  = corridor.size();
    int n_poly = traj_order + 1;
    int s1d1CtrlP_num = n_poly;
    int s1CtrlP_num   = 3 * s1d1CtrlP_num;
    int nbr_nums = 0;
    double Swramrate = 1.0;
    int equ_con_s_num = 3 * 3; // p, v, a in x, y, z axis at the start point
    int equ_con_e_num = 3 * 3; // p, v, a in x, y, z axis at the end point
    int equ_con_continuity_num = 3 * 3 * (segment_num - 1);
    int equ_con_num   = equ_con_s_num + equ_con_e_num + equ_con_continuity_num; // p, v, a in x, y, z axis in each segment's joint position

    int vel_con_num = 3 *  traj_order * segment_num;
    int acc_con_num = 3 * (traj_order - 1) * segment_num;
    double maxVel = highOrderMaxVal[0];
    double maxAcc = highOrderMaxVal[1];

    if( !ENFORCE_VEL )
        vel_con_num = 0;

    if( !ENFORCE_ACC )
        acc_con_num = 0;

    int high_order_con_num = vel_con_num + acc_con_num;
    //int high_order_con_num = 0; //3 * traj_order * segment_num;

    int con_num   = equ_con_num + high_order_con_num;
    int ctrlP_num = segment_num * s1CtrlP_num;

    double x_var[ctrlP_num];
    double primalobj;

    MSKrescodee  r; // 响应代码， 用于报告空间不足等意外故障
    MSKint32t  mskj,mski;
    MSKenv_t   env;
    MSKtask_t  task;
    // mosek program progress 1. 创建环境，2.创建一个优化任务，3.载入一个问题进入任务对象，4.优化，5.获取求解结果

    // Create the mosek environment.
    r = MSK_makeenv( &env, NULL );
    // Create the optimization task.
    r = MSK_maketask(env,con_num, ctrlP_num, &task);

// Parameters used in the optimizer
//######################################################################
    //MSK_putintparam (task, MSK_IPAR_OPTIMIZER , MSK_OPTIMIZER_INTPNT );
    MSK_putintparam (task, MSK_IPAR_NUM_THREADS, 1);
    MSK_putdouparam (task, MSK_DPAR_CHECK_CONVEXITY_REL_TOL, 1e-2);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_TOL_DFEAS,  1e-4);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_TOL_PFEAS,  1e-4);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_TOL_INFEAS, 1e-4);
    //MSK_putdouparam (task, MSK_DPAR_INTPNT_TOL_REL_GAP, 5e-2 );
//######################################################################

    //r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
    // Append empty constraints.
     //The constraints will initially have no bounds.
    if ( r == MSK_RES_OK )
      r = MSK_appendcons(task,con_num);  // 添加 con_num 个空约束，无边界

    // Append optimizing variables. The variables will initially be fixed at zero (x=0).
    if ( r == MSK_RES_OK )
      r = MSK_appendvars(task,ctrlP_num); // 附加 ctrlP_num 个变量并初始化为0

    //ROS_WARN("[Bezier Trajectory] Start stacking the Linear Matrix C, Linear variable part");
    if(USE_SWARM)
    {
        int cjIdx = 0;
        double t_s = 0.01;
        printf("\n");
        for(int ni = 0; ni < (int)nbrInfo.size(); ni ++)
        {
            if(!nbrInfo[ni].live) continue;
            nbr_nums ++;
        }
        // vector<double> CT(s1d1CtrlP_num, 0.0);

        for(int k = 0; k < segment_num; k ++)
        {
            VectorXd CT = VectorXd::Zero(s1d1CtrlP_num);
            MatrixXd H = MatrixXd::Zero((int)nbrInfo.size() ,s1d1CtrlP_num);
            vector<int> nk((int)nbrInfo.size(), -1);
            for(int ni = 0; ni < (int)nbrInfo.size(); ni ++)
            {
                if(!nbrInfo[ni].live) continue;
                nk[ni] = getNbrSegFromTime(nbrInfo[ni].seg_t, nbrInfo[ni].t_duration+t_s);
                // corridor[k].t = max(corridor[k].t, nbrInfo[ni].seg_t(nk[ni]));
            }
            for(int p = 0; p < 3; p ++ )
            {
                for(int ni = 0; ni < (int)nbrInfo.size(); ni ++)
                {
                    if(nk[ni] == -1) continue;
                    // get H
                    for( int hi = 0; hi < s1d1CtrlP_num; hi ++ )
                    {   
                        double hv = 0.0;
                        for(int hj = 0; hj < s1d1CtrlP_num; hj ++)
                        {
                            hv += M(hi, hj) * (nbrInfo[ni].coeff(nk[ni], p*s1d1CtrlP_num+hj));
                        }
                        H(ni, hi) = hv;
                    }
                    // get CT
                    for( int i = 0; i < s1d1CtrlP_num; i ++ )
                    {
                        double my_sj = corridor[k].t, nbr_sj = nbrInfo[ni].seg_t(nk[ni]);
                        double cofb = double(-2.0 * pow(my_sj, 2)) * double (nbrInfo[ni].centerDiff(p)) / double(i+1); 
                        double cofa = 0.0;
                        for( int j = 0; j < s1d1CtrlP_num; j ++ )
                        {
                          cofa += double(-2.0*pow(my_sj, 2)*nbr_sj) * H(ni, j) / double(i+j+1);  
                          if(i >= 1 && j >= 1) cofa += double(-2.0*nbr_sj) *double(i*j) * H(ni, j) / double(i+j-1);
                        }
                        CT(i) += (cofa + cofb);
                    }
                }
                
                // MatrixXd CtM = CT.transpose() * M;
                // insert Ct for variable p
                for( int i = 0; i < s1d1CtrlP_num; i ++ )
                {
                    double CtM = 0.0;
                    for( int l = 0; l < s1d1CtrlP_num; l ++ )
                    {
                        CtM += CT(l) * M(l, i);
                    }
                    if(r == MSK_RES_OK)
                        r = MSK_putcj(task, cjIdx, Swramrate*CtM ); // Swramrate * Jc
                    cjIdx ++;
                }
            }
            t_s += corridor[k].t;
        }

    }

    vector< pair<MSKboundkeye, pair<double, double> > > con_bdk; // boundkey： vec + acc + equ_con
    if(ENFORCE_VEL)
    {
        /***  Stack the bounding value for the linear inequality for the velocity constraints  ***/
        for(int i = 0; i < vel_con_num; i++)
        {
            // MSK_BK_RA： 约束/变量 是有范围的；
            pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_RA, make_pair( - maxVel,  + maxVel) );
            con_bdk.push_back(cb_ie);
        }
    }

    if(ENFORCE_ACC)
    {
        /***  Stack the bounding value for the linear inequality for the acceleration constraints  ***/
        for(int i = 0; i < acc_con_num; i++)
        {
            pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_RA, make_pair( - maxAcc,  maxAcc) );
            con_bdk.push_back(cb_ie);
        }
    }

    //ROS_WARN("[Bezier Trajectory] equality bound %d", equ_con_num);
    for(int i = 0; i < equ_con_num; i ++ ){
        double beq_i;
        if(i < 3)                    beq_i = pos(0, i);
        else if (i >= 3  && i < 6  ) beq_i = vel(0, i - 3);
        else if (i >= 6  && i < 9  ) beq_i = acc(0, i - 6);
        else if (i >= 9  && i < 12 ) beq_i = pos(1, i - 9 );
        else if (i >= 12 && i < 15 ) beq_i = vel(1, i - 12);
        else if (i >= 15 && i < 18 ) beq_i = acc(1, i - 15);
        else beq_i = 0.0;

        // MSK_BK_FX： 约束/变量 是固定的；
        pair<MSKboundkeye, pair<double, double> > cb_eq = make_pair( MSK_BK_FX, make_pair( beq_i, beq_i ) ); // # cb_eq means: constriants boundary of equality constrain
        con_bdk.push_back(cb_eq);
    }

    /* ## define a container for control points' boundary and boundkey ## */
    /* ## dataType in one tuple is : boundary type, lower bound, upper bound ## */
    vector< pair<MSKboundkeye, pair<double, double> > > var_bdk; // 安全区控制点 变量边界值
    for(int k = 0; k < segment_num; k++)
    {
        Cube cube_     = corridor[k];
        double scale_k = cube_.t;

        for(int i = 0; i < 3; i++ )
        {
            for(int j = 0; j < n_poly; j ++ )
            {
                pair<MSKboundkeye, pair<double, double> > vb_x;

                double lo_bound, up_bound;
                if(k > 0)
                {
                    // cube_.box[i].first, cube_.box[i].second
                    lo_bound = (cube_.box[i].first + margin) / scale_k; // margin： 两个corridor，三边交界长度的最小值
                    up_bound = (cube_.box[i].second - margin) / scale_k;
                }
                else
                {
                    lo_bound = (cube_.box[i].first)  / scale_k;
                    up_bound = (cube_.box[i].second) / scale_k;
                }
                // safety constrains in corridor, 对系数的范围约束
                vb_x  = make_pair( MSK_BK_RA, make_pair( lo_bound, up_bound ) );
                // # vb_x means: varialbles boundary of unknowns x (Polynomial coeff)

                var_bdk.push_back(vb_x);
            }
        }
    }

    // ROS_WARN("set variables boundary");
    // { for j=1, ...,ctrlP_num : blx[i] <= var[j] <= bux[i] }
    for(mskj = 0; mskj< ctrlP_num && r == MSK_RES_OK; ++mskj){
        if (r == MSK_RES_OK)
            r = MSK_putvarbound(task,
                                mskj,                            // Index of variable.
                                var_bdk[mskj].first,             // Bound key.
                                var_bdk[mskj].second.first,      // Numerical value of lower bound.
                                var_bdk[mskj].second.second );   // Numerical value of upper bound.
    }

    // Set the bounds on constraints.
    // { for i=1, ...,con_num : blc[i] <= constraint i <= buc[i] }
    for( mski = 0; mski < con_num && r == MSK_RES_OK; mski++ ) {
        r = MSK_putconbound(task,
                            mski,                            // Index of constraint.
                            con_bdk[mski].first,             // Bound key.
                            con_bdk[mski].second.first,      // Numerical value of lower bound.
                            con_bdk[mski].second.second );   // Numerical value of upper bound.
    }

    //ROS_WARN("[Bezier Trajectory] Start stacking the Linear Matrix A, inequality part");
    int row_idx = 0;
    // The velocity constraints
    if(ENFORCE_VEL)
    {
        for(int k = 0; k < segment_num ; k ++ )
        {
            for(int i = 0; i < 3; i++)
            {  // for x, y, z loop
                for(int p = 0; p < traj_order; p++)
                {
                    int nzi = 2;
                    MSKint32t asub[nzi];
                    double aval[nzi];

                    aval[0] = -1.0 * traj_order;
                    aval[1] =  1.0 * traj_order;

                    asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
                    asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;
                    // 通过 MSK_putarow 设置A矩阵的第 row_idx行
                    // MSK_putarow(task, 第 row_idx行, 该行非0元素的个数，该行非0值的行索引(*:传入地址)，该行对应索引的非0值(*:传入地址))
                    r = MSK_putarow(task, row_idx, nzi, asub, aval);
                    row_idx ++;
                }
            }
        }
    }

    // The acceleration constraints
    if(ENFORCE_ACC)
    {
        for(int k = 0; k < segment_num ; k ++ )
        {
            for(int i = 0; i < 3; i++)
            {
                for(int p = 0; p < traj_order - 1; p++)
                {
                    int nzi = 3;
                    MSKint32t asub[nzi];
                    double aval[nzi];

                    aval[0] =  1.0 * traj_order * (traj_order - 1) / corridor[k].t;
                    aval[1] = -2.0 * traj_order * (traj_order - 1) / corridor[k].t;
                    aval[2] =  1.0 * traj_order * (traj_order - 1) / corridor[k].t;
                    asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
                    asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;
                    asub[2] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 2;

                    r = MSK_putarow(task, row_idx, nzi, asub, aval);
                    row_idx ++;
                }
            }
        }
    }

    /*   Start position  */
    {
        // position :
        for(int i = 0; i < 3; i++)
        {  // loop for x, y, z
            int nzi = 1;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = 1.0 * initScale;
            asub[0] = i * s1d1CtrlP_num;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++)
        {  // loop for x, y, z
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = - 1.0 * traj_order;
            aval[1] =   1.0 * traj_order;
            asub[0] = i * s1d1CtrlP_num;
            asub[1] = i * s1d1CtrlP_num + 1;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        // acceleration :
        for(int i = 0; i < 3; i++)
        {  // loop for x, y, z
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] =   1.0 * traj_order * (traj_order - 1) / initScale;
            aval[1] = - 2.0 * traj_order * (traj_order - 1) / initScale;
            aval[2] =   1.0 * traj_order * (traj_order - 1) / initScale;
            asub[0] = i * s1d1CtrlP_num;
            asub[1] = i * s1d1CtrlP_num + 1;
            asub[2] = i * s1d1CtrlP_num + 2;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
    }

    /*   End position  */
    //ROS_WARN(" end position");
    {
        // position :
        for(int i = 0; i < 3; i++)
        {  // loop for x, y, z
            int nzi = 1;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;
            aval[0] = 1.0 * lstScale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++)
        {
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 1;
            asub[1] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;
            aval[0] = - 1.0;
            aval[1] =   1.0;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
        // acceleration :
        for(int i = 0; i < 3; i++)
        {
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 2;
            asub[1] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 1;
            asub[2] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;
            aval[0] =   1.0 / lstScale;
            aval[1] = - 2.0 / lstScale;
            aval[2] =   1.0 / lstScale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);
            row_idx ++;
        }
    }

    /*   joint points continuity constraints */
    //ROS_WARN(" joint position");
    {
        int sub_shift = 0;
        double val0, val1;
        for(int k = 0; k < (segment_num - 1); k ++ )
        {
            double scale_k = corridor[k].t;
            double scale_n = corridor[k+1].t;

            // position :
            val0 = scale_k;
            val1 = scale_n;
            for(int i = 0; i < 3; i++)
            {  // loop for x, y, z
                int nzi = 2;
                MSKint32t asub[nzi];
                double aval[nzi];

                // This segment's last control point
                aval[0] = 1.0 * val0;
                asub[0] = sub_shift + (i+1) * s1d1CtrlP_num - 1;

                // Next segment's first control point
                aval[1] = -1.0 * val1;
                asub[1] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx ++;
            }

            // velocity
            for(int i = 0; i < 3; i++)
            {
                int nzi = 4;
                MSKint32t asub[nzi];
                double aval[nzi];

                // This segment's last velocity control point
                aval[0] = -1.0;
                aval[1] =  1.0;
                asub[0] = sub_shift + (i+1) * s1d1CtrlP_num - 2;
                asub[1] = sub_shift + (i+1) * s1d1CtrlP_num - 1;
                // Next segment's first velocity control point
                aval[2] =  1.0;
                aval[3] = -1.0;

                asub[2] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
                asub[3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx ++;
            }
            // acceleration :
            val0 = 1.0 / scale_k;
            val1 = 1.0 / scale_n;
            for(int i = 0; i < 3; i++)
            {
                int nzi = 6;
                MSKint32t asub[nzi];
                double aval[nzi];

                // This segment's last velocity control point
                aval[0] =  1.0  * val0;
                aval[1] = -2.0  * val0;
                aval[2] =  1.0  * val0;
                asub[0] = sub_shift + (i+1) * s1d1CtrlP_num - 3;
                asub[1] = sub_shift + (i+1) * s1d1CtrlP_num - 2;
                asub[2] = sub_shift + (i+1) * s1d1CtrlP_num - 1;
                // Next segment's first velocity control point
                aval[3] =  -1.0  * val1;
                aval[4] =   2.0  * val1;
                aval[5] =  -1.0  * val1;
                asub[3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
                asub[4] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;
                asub[5] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 2;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);
                row_idx ++;
            }

            sub_shift += s1CtrlP_num;
        }
    }

    //ROS_WARN("[Bezier Trajectory] Start stacking the objective");
    int min_order_l = floor(minimize_order);
    int min_order_u = ceil (minimize_order);
    int NUMQNZ = 0;
    for(int i = 0; i < segment_num; i ++)
    {
        int NUMQ_blk = (traj_order + 1);                       // default minimize the jerk and minimize_order = 3
        NUMQNZ      += 3 * NUMQ_blk * (NUMQ_blk + 1) / 2;
    }
    MSKint32t  qsubi[NUMQNZ], qsubj[NUMQNZ];
    double     qval[NUMQNZ];

    // set objective QP Matrix
    {
        int sub_shift = 0;
        int idx = 0;
        for(int k = 0; k < segment_num; k ++)
        {
            double scale_k = corridor[k].t;
            for(int p = 0; p < 3; p ++ )
                for( int i = 0; i < s1d1CtrlP_num; i ++ )
                    for( int j = 0; j < s1d1CtrlP_num; j ++ )
                        if( i >= j )
                        {
                            qsubi[idx] = sub_shift + p * s1d1CtrlP_num + i;
                            qsubj[idx] = sub_shift + p * s1d1CtrlP_num + j;
                            //qval[idx]  = MQM(i, j) /(double)pow(Tk, 3);
                            if(min_order_l == min_order_u)
                                qval[idx]  = MQM(i, j) /(double)pow(scale_k, 2 * min_order_u - 3); // (double)pow(scale_k, 2 * min_order_u - 3);
                                // (double)pow(scale_k, i + j - 2 * min_order_u + 1)
                            else
                                qval[idx] = ( (minimize_order - min_order_l) / (double)pow(scale_k, 2 * min_order_u - 3)
                                            + (min_order_u - minimize_order) / (double)pow(scale_k, 2 * min_order_l - 3) ) * MQM(i, j);
                            if(USE_SWARM) {
                                qval[idx] += (double)nbr_nums * Swramrate * MGM(i,j) * (double)pow(scale_k, 3); //  /t^(2 * 0 - 3)
                                qval[idx] += (double)nbr_nums * Swramrate * MVM(i,j) * (double)scale_k; // /t^(2 * 1 - 3)
                            }
                            idx ++ ;
                        }

            sub_shift += s1CtrlP_num;
        }
    }

    ros::Time time_end1 = ros::Time::now();

    if ( r== MSK_RES_OK )
         r = MSK_putqobj(task,NUMQNZ,qsubi,qsubj,qval); //

    if ( r==MSK_RES_OK )
         r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE); // 问题被最小化

    //ros::Time time_opt = ros::Time::now();
    bool solve_ok = false;
    if ( r==MSK_RES_OK )
      {
        //ROS_WARN("Prepare to solve the problem ");
        MSKrescodee trmcode;
        r = MSK_optimizetrm(task,&trmcode);
        MSK_solutionsummary (task,MSK_STREAM_LOG);

        if ( r==MSK_RES_OK )
        {
          MSKsolstae solsta;
          MSK_getsolsta (task,MSK_SOL_ITR,&solsta); // get solution status

          switch(solsta)
          {
            case MSK_SOL_STA_OPTIMAL:
            case MSK_SOL_STA_NEAR_OPTIMAL:


            r = MSK_getxx(task,
                          MSK_SOL_ITR,    // Request the interior solution.
                          x_var); // 原变量解

            r = MSK_getprimalobj(
                task,
                MSK_SOL_ITR,
                &primalobj); // 目标值

            obj = primalobj;
            solve_ok = true;

            break;

            case MSK_SOL_STA_DUAL_INFEAS_CER:
            case MSK_SOL_STA_PRIM_INFEAS_CER:
            case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
            case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
              printf("Primal or dual infeasibility certificate found.\n");
              break;

            case MSK_SOL_STA_UNKNOWN:
              printf("The status of the solution could not be determined.\n");
              //solve_ok = true; // debug
              break;
            default:
              printf("Other solution status.");
              break;
          }
        }
        else
        {
          printf("Error while optimizing.\n");
        }
      }

      if (r != MSK_RES_OK)
      {
        // In case of an error print error code and description.
        char symname[MSK_MAX_STR_LEN];
        char desc[MSK_MAX_STR_LEN];

        printf("An error occurred while optimizing.\n");
        MSK_getcodedesc (r,
                         symname,
                         desc);
        printf("Error %s - '%s'\n",symname,desc);
      }

    MSK_deletetask(&task);
    MSK_deleteenv(&env);

    ros::Time time_end2 = ros::Time::now();
    // ROS_WARN("time consume in optimize is:  %f;", (time_end2 - time_end1).toSec());
    printf("\033[32m[solveQP]\033[0m time consume in optimize is %f; ", (time_end2 - time_end1).toSec());

    // cout<<time_end2 - time_end1<<endl;

    if(!solve_ok){
      printf("\033[31mIn solver, falied.\033[0m ");
      return -1;
    }

    // pub best answer to PolyCoeff
    VectorXd d_var(ctrlP_num);
    for(int i = 0; i < ctrlP_num; i++)
        d_var(i) = x_var[i];

    PolyCoeff = MatrixXd::Zero(segment_num, 3 *(traj_order + 1) );

    int var_shift = 0;
    for(int i = 0; i < segment_num; i++ )
    {
        for(int j = 0; j < 3 * n_poly; j++)
            PolyCoeff(i , j) = d_var(j + var_shift);

        var_shift += 3 * n_poly;
    }

    return 1;
}

quadrotor_msgs::PolynomialTrajectory TrajectoryGenerator::getBezierTraj(const MatrixXd & polyCoeff, const VectorXd & _seg_time, int _seg_num, int _traj_id, ros::Time t_now )
{
    quadrotor_msgs::PolynomialTrajectory traj;
      traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
      traj.num_segment = _seg_num;

      int order = traj_order;
      int poly_num1d = order + 1;
      int polyTotalNum = _seg_num * (order + 1);

      traj.coef_x.resize(polyTotalNum);
      traj.coef_y.resize(polyTotalNum);
      traj.coef_z.resize(polyTotalNum);

      int idx = 0;
      for(int i = 0; i < _seg_num; i++ )
      {
          for(int j =0; j < poly_num1d; j++)
          {
              traj.coef_x[idx] = polyCoeff(i,                  j);
              traj.coef_y[idx] = polyCoeff(i,     poly_num1d + j);
              traj.coef_z[idx] = polyCoeff(i, 2 * poly_num1d + j);
              idx++;
          }
      }

      traj.header.frame_id = "/bernstein";
      traj.header.stamp = t_now;
      traj.time.resize(_seg_num);
      traj.order.resize(_seg_num);

      traj.mag_coeff = 1.0;
      for (int idx = 0; idx < _seg_num; ++idx){
          traj.time[idx] = _seg_time(idx);
          traj.order[idx] = order;
      }

    //   Vector3d initialVel, finalVel;
    //   initialVel = getVelFromBezier(polyCoeff, _Cv, traj_order, 0, 0);
    //   finalVel = getVelFromBezier(polyCoeff, _Cv, traj_order, traj.time[traj.num_segment - 1] ,traj.num_segment - 1);

    //   traj.start_yaw = atan2(initialVel(1), initialVel(0));
    //   traj.final_yaw = atan2(finalVel(1), finalVel(0));

      traj.start_yaw = 0.0;
      traj.final_yaw = 0.0;
      traj.trajectory_id = _traj_id;
      traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

      return traj;
}

Vector3d TrajectoryGenerator::getPosFromBezier(const MatrixXd & polyCoeff, const VectorXd & _C,  int _traj_order, double t_now, int seg_now )
{
    Vector3d ret = VectorXd::Zero(3);
    VectorXd ctrl_now = polyCoeff.row(seg_now);
    int ctrl_num1D = polyCoeff.cols() / 3;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < ctrl_num1D; j++)
            ret(i) += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (_traj_order - j) );
    return ret;
}

Vector3d TrajectoryGenerator::getPosFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now )
{
    Vector3d ret = VectorXd::Zero(3);
    VectorXd ctrl_now = polyCoeff.row(seg_now);
    int ctrl_num1D = polyCoeff.cols() / 3;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < ctrl_num1D; j++)
            ret(i) += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (traj_order - j) );
    return ret;
}

Vector3d TrajectoryGenerator::getVelFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now) {
  Vector3d ret = VectorXd::Zero(3);
  VectorXd ctrl_now = polyCoeff.row(seg_now);
  int ctrl_num1D = (int)polyCoeff.cols() / 3;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < ctrl_num1D; j++)
        ret(i) += _Cv(j) * traj_order * ( ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (traj_order - j - 1) );

  return ret;
}

VectorXd TrajectoryGenerator::getStateFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now )
{
    VectorXd ret = VectorXd::Zero(12);

    VectorXd ctrl_now = polyCoeff.row(seg_now);
    int ctrl_num1D = polyCoeff.cols() / 3;

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < ctrl_num1D; j++){
            ret[i] += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (traj_order - j) );

            if(j < ctrl_num1D - 1 )
                ret[i+3] += _Cv(j) * traj_order
                      * ( ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (traj_order - j - 1) );

            if(j < ctrl_num1D - 2 )
                ret[i+6] += _Ca(j) * traj_order * (traj_order - 1)
                      * ( ctrl_now(i * ctrl_num1D + j + 2) - 2 * ctrl_now(i * ctrl_num1D + j + 1) + ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (traj_order - j - 2) );

            if(j < ctrl_num1D - 3 )
                ret[i+9] += _Cj(j) * traj_order * (traj_order - 1) * (traj_order - 2)
                      * ( ctrl_now(i * ctrl_num1D + j + 3) - 3 * ctrl_now(i * ctrl_num1D + j + 2) + 3 * ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (traj_order - j - 3) );
        }
    }

    return ret;
}

Vector3d TrajectoryGenerator::getTsPosFromBezier(const MatrixXd & polyCoeff, const VectorXd & seg_time, double t_s)
{
    Vector3d ret = VectorXd::Zero(3);
    int seg_num = (int)seg_time.rows(), idx = 0;
    double t = t_s;

    for (idx = 0; idx < seg_num; ++idx)
    {
        if( t  > seg_time(idx) && idx + 1 < seg_num)
            t -= seg_time(idx);
        else
            break;
    }
    if(idx >= seg_num){
        ROS_WARN("[Bezier Trajectory] out nbr total seg time, may cause error result");
        idx = seg_num-1;
    }
    Vector3d state = VectorXd::Zero(3);
    state = getPosFromBezier( polyCoeff, t/seg_time(idx), idx);
    ret(0) = seg_time(idx) * state(0);
    ret(1) = seg_time(idx) * state(1);
    ret(2) = seg_time(idx) * state(2);

    return ret;
}

int TrajectoryGenerator::getNbrSegFromTime(const VectorXd & seg_time, double t_s)
{
    int seg_num = (int)seg_time.rows();
    if(seg_num == 0) return -1;
    int idx = 0;
    // printf("\033[34m[getNbrSegFromTime] seg_time = %d * %d, t_s = %f\033[0m. ", (int)seg_time.rows(), (int)seg_time.cols(), t_s);
    double t = t_s;

    for (idx = 0; idx < seg_num; ++idx)
    {
        // printf("seg_time[%d] = %f; ",idx, seg_time(idx) );
        if(t > seg_time(idx)*0.75) // t  > seg_time(idx)
        {
            t -= seg_time(idx);
        }
        else{
            break;
        }
    }
    if(idx >= seg_num){
        printf("\033[33m[Bezier Trajectory] out nbr total seg time, may cause error result\033[0m. ");
        idx = -1;
    }
    // printf("get %d !!!!!!!!!\n", idx);
    return idx;
}
