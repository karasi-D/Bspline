#ifndef _DATA_TYPE_
#define _DATA_TYPE_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <math.h>
#include <sdf_tools/collision_map.hpp>

using namespace std;
using namespace Eigen;

#define inf 1>>30

struct Cube;
struct GridNode;
struct Nbr_Info;
typedef GridNode* GridNodePtr;

struct Cube
{     
      //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;    
      Eigen::MatrixXd vertex; // the 8 vertex of a cube
      Eigen::Vector3d center; // the center of the cube
      bool valid;    // indicates whether this cube should be deleted

      double t; // time allocated to this cube
      std::vector< std::pair<double, double> > box;
/*
           P4------------P3 
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /  
        P5------------P6              / x
*/                                                                                 

      // create a cube using 8 vertex and the center point
      Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(3);
      }

      // create a inscribe cube of a ball using the center point and the radius of the ball
      void setVertex( Eigen::MatrixXd vertex_, double resolution_)
      {     
            vertex = vertex_;
            vertex(0,1) -= resolution_ / 2.0;
            vertex(3,1) -= resolution_ / 2.0;
            vertex(4,1) -= resolution_ / 2.0;
            vertex(7,1) -= resolution_ / 2.0;

            vertex(1,1) += resolution_ / 2.0;
            vertex(2,1) += resolution_ / 2.0;
            vertex(5,1) += resolution_ / 2.0;
            vertex(6,1) += resolution_ / 2.0;

            vertex(0,0) += resolution_ / 2.0;
            vertex(1,0) += resolution_ / 2.0;
            vertex(4,0) += resolution_ / 2.0;
            vertex(5,0) += resolution_ / 2.0;

            vertex(2,0) -= resolution_ / 2.0;
            vertex(3,0) -= resolution_ / 2.0;
            vertex(6,0) -= resolution_ / 2.0;
            vertex(7,0) -= resolution_ / 2.0;

            vertex(0,2) += resolution_ / 2.0;
            vertex(1,2) += resolution_ / 2.0;
            vertex(2,2) += resolution_ / 2.0;
            vertex(3,2) += resolution_ / 2.0;

            vertex(4,2) -= resolution_ / 2.0;
            vertex(5,2) -= resolution_ / 2.0;
            vertex(6,2) -= resolution_ / 2.0;
            vertex(7,2) -= resolution_ / 2.0;
            
            setBox();
      }
      
      void setBox()
      {
            box.clear();
            box.resize(3);
            box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) );
            box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) );
            box[2] = std::make_pair( vertex(4, 2), vertex(1, 2) );
      }

      void printBox()
      {
            std::cout<<"center of the cube: \n"<<center<<std::endl;
            std::cout<<"vertex of the cube: \n"<<vertex<<std::endl;
      }

      Cube()
      {  
         center = Eigen::VectorXd::Zero(3);
         vertex = Eigen::MatrixXd::Zero(8, 3);

         valid = true;
         t = 0.0;
         box.resize(3);
      }

      ~Cube(){}
};

struct GridNode
{     
   int id;        // 1--> open set, -1 --> closed set
   Eigen::Vector3d coord;
   Eigen::Vector3i index;
   
   double gScore, fScore;
   GridNodePtr cameFrom;
   std::multimap<double, GridNodePtr>::iterator nodeMapIt;
   double occupancy, obsDis; 

   std::vector<GridNodePtr> hisNodeList; // use a list to record nodes in its history

   GridNode(Eigen::Vector3i _index)
   {  
      id = 0;
      index = _index;
      
      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
   }

   GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
   {  
      id = 0;
      index = _index;
      coord = _coord;

      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
   }

   GridNode(){};
   
   ~GridNode(){};
};

struct Nbr_Info{
      int id;
      int issue;
      bool live;
      double t_duration;
      std::vector<Eigen::Vector3d> path_coord;
      Eigen::MatrixXd coeff;
      Eigen::VectorXd seg_t;
      Eigen::Vector3d centerDiff;
      Eigen::Vector3d pos;
      Eigen::Vector3d vel;
      // Eigen::Vector3d acc;
};

class CollisionGrid{
private:
      Eigen::Affine3d origin_transform_;
      Eigen::Affine3d inverse_origin_transform_;
public:
      int8_t * data;
      std::string frame;
      Eigen::Vector3i goalIdx;
      int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
      int GLXYZ_SIZE, GLYZ_SIZE;
      double _resolution, _inv_resolution;

      CollisionGrid(){}
      ~CollisionGrid(){}

      CollisionGrid(const Eigen::Affine3d& origin_transform, const std::string& frame, const double resolution, const int max_x_id,
                                  const int max_y_id, const int max_z_id){
            this->frame = frame;
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();

            _resolution = resolution;
            _inv_resolution = 1.0 / _resolution;
            GLX_SIZE = max_x_id;
            GLY_SIZE = max_y_id;
            GLZ_SIZE = max_z_id;
            GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
            GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

            data = new int8_t[GLXYZ_SIZE];
            memset(data, 0, GLXYZ_SIZE * sizeof(int8_t));
      }

      inline void RestMap(){
            memset(data, 0, GLXYZ_SIZE * sizeof(int8_t));
      }
      inline int64_t GetDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
      {
            return (x_index * GLYZ_SIZE) + (y_index * GLZ_SIZE) + z_index;
      }

      inline bool IndexInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
      {
            if (x_index >= 0 && y_index >= 0 && z_index >= 0 && x_index < GLX_SIZE && y_index < GLY_SIZE && z_index < GLZ_SIZE)
            {
                  return true;
            }
            else
            {
                  return false;
            }
      }
      
      inline Eigen::Vector3i LocationToGridIndex(const Eigen::Vector3d& location) const
      {
            const Eigen::Vector4d point(location(0), location(1), location(2), 1.0);
            const Eigen::Vector4d point_in_grid_frame = inverse_origin_transform_ * point;
            const int64_t x_cell = (int64_t)(point_in_grid_frame(0) * _inv_resolution);
            const int64_t y_cell = (int64_t)(point_in_grid_frame(1) * _inv_resolution);
            const int64_t z_cell = (int64_t)(point_in_grid_frame(2) * _inv_resolution);

            Eigen::Vector3i index(x_cell, y_cell, z_cell);   
            return index;
      }

      inline Eigen::Vector3d GridIndexToLocation(const Eigen::Vector3i index) const
      {    
            const Eigen::Vector3d point_in_grid_frame(_resolution * ((double)index(0) + 0.5), _resolution * ((double)index(1) + 0.5), _resolution * ((double)index(2) + 0.5));
            return origin_transform_ * point_in_grid_frame;            
      }

      inline bool setObs (const Eigen::Vector3d &location, const int8_t& value)
      {
            const Eigen::Vector3d point_in_grid_frame = inverse_origin_transform_ * location;
            const int64_t x_cell = (int64_t)(point_in_grid_frame.x() * _inv_resolution);
            const int64_t y_cell = (int64_t)(point_in_grid_frame.y() * _inv_resolution);
            const int64_t z_cell = (int64_t)(point_in_grid_frame.z() * _inv_resolution);
            if (IndexInBounds(x_cell, y_cell, z_cell)){
                  const int64_t data_index = GetDataIndex(x_cell, y_cell, z_cell);
                  assert(data_index >= 0 && data_index < GLXYZ_SIZE);
                  data[data_index] = value;
                  return true;
            }
            else
                  return false;
      }

      inline std::pair<int8_t, bool>  Get(const int64_t x_index, const int64_t y_index, const int64_t z_index)
      {
            if (IndexInBounds(x_index, y_index, z_index))
            {
                  const int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                  assert(data_index >= 0 && data_index < GLXYZ_SIZE);
                  return std::pair<const int8_t&, bool>(data[data_index], true);
            }
            else
            {
                  return std::pair<const int8_t&, bool>(1, false);
            }
      }
};

class CorridorGenerate{
private:
      double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
      int _max_x_id, _max_y_id, _max_z_id;
      double _resolution, _horizon;
      int _max_inflate_iter, _step_length;
      int uav_id;

public:
      CorridorGenerate(){}
      ~CorridorGenerate(){}
      void setParam(int id,  double horizon){
            uav_id = id;
            _horizon = horizon;
      }
      CorridorGenerate(Vector3d & pt_min, Vector3d & pt_max, Vector3i & id_max, double resolution, int max_inflate_iter, int step_length)
      : _resolution(resolution), _max_inflate_iter(max_inflate_iter), _step_length(step_length)
      {
           _pt_min_x = pt_min(0);
           _pt_min_y = pt_min(1);
           _pt_min_z = pt_min(2);
           _pt_max_x = pt_max(0);
           _pt_max_y = pt_max(1);
           _pt_max_z = pt_max(2);
           _max_x_id = id_max(0);
           _max_y_id = id_max(1);
           _max_z_id = id_max(2);
           
      };
      
      Cube generateCube( Vector3d pt, const sdf_tools::CollisionMapGrid * collision_map); 
      bool isContains(Cube cube1, Cube cube2);
      pair<Cube, bool> inflateCube(Cube cube, Cube lstcube, const sdf_tools::CollisionMapGrid * collision_map);
      std::vector<Cube> corridorGeneration(const vector<Vector3d> & path_coord, const vector<double> & time, const sdf_tools::CollisionMapGrid * collision_map);
      std::vector<Cube> corridorGeneration(const vector<Vector3d> & path_coord, const sdf_tools::CollisionMapGrid * collision_map);
      void corridorSimplify(vector<Cube> & cubicList, const sdf_tools::CollisionMapGrid * collision_map);
      bool generatePath(const vector<Cube> & cubecList, MatrixXd& local_path);

};

#endif