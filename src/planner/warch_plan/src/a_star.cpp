#include "a_star.h"

using namespace std;
using namespace Eigen;
using namespace sdf_tools;

void gridPathFinder::initGridNodeMap(double _resolution, Vector3d global_xyz_l)
{   
    _world_origin = global_xyz_l;
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
       GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
       for(int j = 0; j < GLY_SIZE; j++){
           GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
           for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
           }
       }
    }
}

// 梯度场 计算当前点的障碍值 
double gridPathFinder::getObsDisCost(double dis, double max_value)
{
    double cost = 0.0;
    
    if(max_value >= 10.0){
        /*1. h(n) for dis*/
        if(dis <= 0){
            cost = 10.0;
        }
        else if(dis > 0 && dis < 0.25)
            // cost = 3.0 / (1+exp(10.0*dis));
            cost = 2.0* (4*exp(-10.0*dis) / ((1+exp(-10.0*dis)) * (1+exp(-10.0*dis))));
        else if(dis >= 0.25 && dis < 0.5) 
            cost = 1.0* (4*exp(-5.0*dis) / ((1+exp(-5.0*dis)) * (1+exp(-5.0*dis))));
        else if(dis >= 0.5) 
            cost = -(dis - 0.5) / (1+exp(-1.0*dis));
    }
    else if(max_value < 0){
        /*1. g(n) for dis*/
        if(dis < 0) 
            cost = 5.0*exp(-0.1*dis)-3.5;
        else if(dis >= 0 && dis < 1.0)
            cost = 0.1*log(dis)/log(0.5);
        else if(dis >= 1.0)
            cost = 0.5*log(dis)/log(0.5);
    }else{
        /*2. for delta_dis*/
        if(dis < 0)
            cost = 200*max_value * (dis*dis);
            // cost = 3*max_value / (1+exp(10.0*dis));
            // cost = exp(10.0*dis) / ((1+exp(10.0*dis)) * (1+exp(10.0*dis))) *2*max_value;
        else if(dis > 0 && dis < max_value) 
            cost = -1.0*max_value / (1+exp(-1.0*dis));
    }
    return cost ;
}

void gridPathFinder::linkLocalMap(CollisionMapGrid * local_map, Vector3d xyz_l)
{    
    // calculate EDT map
    ros::Time time_1 = ros::Time::now();
    float oob_value = INFINITY;
    auto EDT = local_map->ExtractDistanceField(oob_value);
    ros::Time time_2 = ros::Time::now();
    printf("\033[32m[UAV %d - EDT]\033[0m time in generate EDT is %f; ", uav_id, (time_2 - time_1).toSec());

    // set a* nodeMap
    Vector3d coord; 
    for(int64_t i = 0; i < X_SIZE; i++)
    {
        for(int64_t j = 0; j < Y_SIZE; j++)
        {
            for(int64_t k = 0; k < Z_SIZE; k++)
            {   
                coord(0) = xyz_l(0) + (double)(i + 0.5) * resolution;
                coord(1) = xyz_l(1) + (double)(j + 0.5) * resolution;
                coord(2) = xyz_l(2) + (double)(k + 0.5) * resolution;

                Vector3i index = coord2gridIndex(coord);

                if( index(0) >= GLX_SIZE || index(1) >= GLY_SIZE || index(2) >= GLZ_SIZE 
                 || index(0) <  0 || index(1) < 0 || index(2) <  0 )
                    continue;
                GridNodePtr ptr = GridNodeMap[index(0)][index(1)][index(2)];
                ptr->id = 0;
                ptr->occupancy = local_map->Get(i, j, k ).first.occupancy;

                Vector3d worldPt;
                // worldPt(0) = (i + 0.5) * resolution + xyz_l(0);
                // worldPt(1) = (j + 0.5) * resolution + xyz_l(1);
                // worldPt(2) = (k + 0.5) * resolution + xyz_l(2);

                worldPt(0) = (i + 0.5) * resolution + _world_origin(0);
                worldPt(1) = (j + 0.5) * resolution + _world_origin(1);
                worldPt(2) = (k + 0.5) * resolution + _world_origin(2);
                Vector3i worldIndex = local_map->LocationToGridIndex(worldPt);
                double nearDis = 0.0;
                if(local_map->Inside(worldIndex))
                {
                    nearDis = sqrt(EDT.GetImmutable(worldIndex).first.distance_square) * resolution;
                }
                // if( k == 0 || k == (Z_SIZE - 1) || j == 0 || j == (Y_SIZE - 1) || i == 0 || i == (X_SIZE - 1) )
                //     nearDis = 2.0;
                ptr->obsDis = nearDis;
            }
        }
    }
}

void gridPathFinder::resetLocalMap()
{   
    //ROS_WARN("expandedNodes size : %d", expandedNodes.size());
    for(auto tmpPtr:expandedNodes)
    {
        tmpPtr->occupancy = 0; // forget the occupancy
        tmpPtr->obsDis = 0.0;
        tmpPtr->id = 0;
        tmpPtr->cameFrom = NULL;
        tmpPtr->gScore = inf;
        tmpPtr->fScore = inf;
    }

    for(auto ptr:openSet)
    {   
        GridNodePtr tmpPtr = ptr.second;
        tmpPtr->occupancy = 0; // forget the occupancy
        tmpPtr->obsDis = 0.0;
        tmpPtr->id = 0;
        tmpPtr->cameFrom = NULL;
        tmpPtr->gScore = inf;
        tmpPtr->fScore = inf;
    }

    expandedNodes.clear();
    //ROS_WARN("local map reset finish");
}

GridNodePtr gridPathFinder::pos2gridNodePtr(Vector3d pos)
{
    Vector3i idx = coord2gridIndex(pos);
    GridNodePtr grid_ptr = new GridNode(idx, pos);

    return grid_ptr;
}

Vector3d gridPathFinder::gridIndex2coord(Vector3i index)
{
    Vector3d pt;
    //cell_x_size_ * ((double)x_index + 0.5), cell_y_size_ * ((double)y_index + 0.5), cell_z_size_ * ((double)z_index + 0.5)

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    /*pt(0) = (double)index(0) * resolution + gl_xl + 0.5 * resolution;
    pt(1) = (double)index(1) * resolution + gl_yl + 0.5 * resolution;
    pt(2) = (double)index(2) * resolution + gl_zl + 0.5 * resolution;*/
    return pt;
}

Vector3i gridPathFinder::coord2gridIndex(Vector3d pt)
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);      

    return idx;
}

double gridPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double gridPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double gridPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{   
    return (node2->index - node1->index).norm();
}

double gridPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return getDiagHeu(node1, node2);
    // return tie_breaker * getDiagHeu(node1, node2);
    //return tie_breaker * getEuclHeu(node1, node2);
}

vector<GridNodePtr> gridPathFinder::retrievePath(GridNodePtr currentPtr)
{   
    vector<GridNodePtr> path;
    path.push_back(currentPtr);

    while(currentPtr->cameFrom != NULL)
    {
        currentPtr = currentPtr -> cameFrom;
        path.push_back(currentPtr);
    }

    return path;
}

vector<GridNodePtr> gridPathFinder::getVisitedNodes()
{   
    vector<GridNodePtr> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++)
            {   
                if(GridNodeMap[i][j][k]->id != 0)
                //if(GridNodeMap[i][j][k]->id == -1)
                    visited_nodes.push_back(GridNodeMap[i][j][k]);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/*bool gridPathFinder::minClearance()
{
    neighborPtr->occupancy > 0.5
}
*/
void gridPathFinder::AstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    
    GridNodePtr startPtr = pos2gridNodePtr(start_pt);
    GridNodePtr endPtr   = pos2gridNodePtr(end_pt);

    openSet.clear();

    GridNodePtr neighborPtr = NULL;
    GridNodePtr currentPtr = NULL;

    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr, endPtr);
    startPtr -> id = 1; //put start node in open set
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); //put start in open set

    double tentative_gScore;
    int num_iter = 0;
    while ( !openSet.empty() )
    {   
        num_iter ++;
        currentPtr = openSet.begin() -> second;

        if(currentPtr->index(0) == endPtr->index(0)
        && currentPtr->index(1) == endPtr->index(1)
        && currentPtr->index(2) == endPtr->index(2) )
        {
            ROS_WARN("[Astar]Reach goal..");
            //cout << "goal coord: " << endl << currentPtr->real_coord << endl; 
            cout << "total number of iteration used in Astar: " << num_iter  << endl;
            ros::Time time_2 = ros::Time::now();
            ROS_WARN("[Astar] Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath = retrievePath(currentPtr);
            return;
        }         
        openSet.erase(openSet.begin());
        currentPtr -> id = -1; //move currentPtr node from open set to closed set.
        expandedNodes.push_back(currentPtr);

        for(int dx = -1; dx < 2; dx++)
            for(int dy = -1; dy < 2; dy++)
                for(int dz = -1; dz < 2; dz++)
                {
                    if(dx == 0 && dy == 0 && dz ==0)
                        continue; 

                    Vector3i neighborIdx;
                    neighborIdx(0) = (currentPtr -> index)(0) + dx;
                    neighborIdx(1) = (currentPtr -> index)(1) + dy;
                    neighborIdx(2) = (currentPtr -> index)(2) + dz;

                    if(    neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE
                        || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE
                        || neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE){
                        continue;
                    }

                    neighborPtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];

/*                    if(minClearance() == false){
                        continue;
                    }*/

                    if(neighborPtr-> occupancy > 0.5){
                        continue;
                    }

                    if(neighborPtr -> id == -1){
                        continue; //in closed set.
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    
                    tentative_gScore = currentPtr -> gScore + static_cost; // ; 

                    if(neighborPtr -> id != 1){
                        //discover a new node
                        neighborPtr -> id        = 1;
                        neighborPtr -> cameFrom  = currentPtr;
                        neighborPtr -> gScore    = tentative_gScore;
                        // neighborPtr -> fScore    = neighborPtr -> gScore + tie_breaker * getHeu(neighborPtr, endPtr) + getObsDisCost(neighborPtr->obsDis - currentPtr->obsDis, static_cost);  // delta_dis
                        neighborPtr -> fScore    = neighborPtr -> gScore + (tie_breaker + getObsDisCost(neighborPtr->obsDis, 10.0)) * getHeu(neighborPtr, endPtr) ; //dis
                        // neighborPtr -> fScore    = (neighborPtr -> gScore + tie_breaker * getHeu(neighborPtr, endPtr)) *(1.0+getObsDisCost(neighborPtr->obsDis, 10.0)); //dis
                        neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                        continue;
                    }
                    else if(tentative_gScore <= neighborPtr-> gScore){ //in open set and need update
                        neighborPtr -> cameFrom = currentPtr;
                        neighborPtr -> gScore = tentative_gScore;
                        // neighborPtr -> fScore = tentative_gScore + tie_breaker * getHeu(neighborPtr, endPtr) + getObsDisCost(neighborPtr->obsDis - currentPtr->obsDis, static_cost); 
                        neighborPtr -> fScore    = neighborPtr -> gScore + (tie_breaker + getObsDisCost(neighborPtr->obsDis, 10.0)) * getHeu(neighborPtr, endPtr) ; //dis
                        // neighborPtr -> fScore    = (neighborPtr -> gScore + tie_breaker * getHeu(neighborPtr, endPtr)) *(1.0+getObsDisCost(neighborPtr->obsDis, 10.0)); //dis
                        openSet.erase(neighborPtr -> nodeMapIt);
                        neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                    }
                        
                }
    }

    ros::Time time_2 = ros::Time::now();
    ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
}

vector<Vector3d> gridPathFinder::getPath()
{   
    vector<Vector3d> path;

    for(auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());
    return path;
}

void gridPathFinder::resetPath()
{
    gridPath.clear();
}