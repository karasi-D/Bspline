#include "data_type.h"
using namespace std;    
using namespace Eigen;
using namespace sdf_tools;

Cube CorridorGenerate::generateCube( Vector3d pt, const sdf_tools::CollisionMapGrid * collision_map) 
{   
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
    Cube cube;
    
    pt(0) = max(min(pt(0), _pt_max_x), _pt_min_x);
    pt(1) = max(min(pt(1), _pt_max_y), _pt_min_y);
    pt(2) = max(min(pt(2), _pt_max_z), _pt_min_z);

    Vector3i pc_index = collision_map->LocationToGridIndex(pt);    
    Vector3d pc_coord = collision_map->GridIndexToLocation(pc_index);

    cube.center = pc_coord;
    double x_u = pc_coord(0);
    double x_l = pc_coord(0);
    
    double y_u = pc_coord(1);
    double y_l = pc_coord(1);
    
    double z_u = pc_coord(2);
    double z_l = pc_coord(2);

    cube.vertex.row(0) = Vector3d(x_u, y_l, z_u);  
    cube.vertex.row(1) = Vector3d(x_u, y_u, z_u);  
    cube.vertex.row(2) = Vector3d(x_l, y_u, z_u);  
    cube.vertex.row(3) = Vector3d(x_l, y_l, z_u);  

    cube.vertex.row(4) = Vector3d(x_u, y_l, z_l);  
    cube.vertex.row(5) = Vector3d(x_u, y_u, z_l);  
    cube.vertex.row(6) = Vector3d(x_l, y_u, z_l);  
    cube.vertex.row(7) = Vector3d(x_l, y_l, z_l);  

    return cube;
}

bool CorridorGenerate::isContains(Cube cube1, Cube cube2)
{   
    if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(0, 1) && cube1.vertex(0, 2) >= cube2.vertex(0, 2) &&
        cube1.vertex(6, 0) <= cube2.vertex(6, 0) && cube1.vertex(6, 1) >= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(6, 2)  )
        return true;
    else
        return false; 
}

pair<Cube, bool> CorridorGenerate::inflateCube(Cube cube, Cube lstcube, const sdf_tools::CollisionMapGrid * collision_map)
{   
    Cube cubeMax = cube;
    
    // Inflate sequence: left, right, front, back, below, above                                                                                
    MatrixXi vertex_idx(8, 3);
    for (int i = 0; i < 8; i++)
    { 
        double coord_x = max(min(cube.vertex(i, 0), _pt_max_x), _pt_min_x);
        double coord_y = max(min(cube.vertex(i, 1), _pt_max_y), _pt_min_y);
        double coord_z = max(min(cube.vertex(i, 2), _pt_max_z), _pt_min_z);
        Vector3d coord(coord_x, coord_y, coord_z);
        Vector3i pt_idx = collision_map->LocationToGridIndex(coord);

        if( collision_map->Get( (int64_t)pt_idx(0), (int64_t)pt_idx(1), (int64_t)pt_idx(2) ).first.occupancy > 0.5 )
        {       
            ROS_ERROR("[UAV %d - inflateCube] path has node in obstacles !", uav_id);
            return make_pair(cubeMax, false);
        }
        
        vertex_idx.row(i) = pt_idx;
    }

    int id_x, id_y, id_z;

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

    bool collide;
    MatrixXi vertex_idx_lst = vertex_idx;

    int iter = 0;
    while(iter < _max_inflate_iter)
    { 
        // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face sweep
        // ############################################################################################################
         
        int y_lo = max(0, vertex_idx(0, 1) - _step_length);
        int y_up = min(_max_y_id, vertex_idx(1, 1) + _step_length);

        collide  = false;
        for(id_y = vertex_idx(0, 1); id_y >= y_lo; id_y-- )
        {   
            if( collide == true) 
                break;
            
            for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
            {    
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 1) = min(id_y+2, vertex_idx(0, 1));
            vertex_idx(3, 1) = min(id_y+2, vertex_idx(3, 1));
            vertex_idx(7, 1) = min(id_y+2, vertex_idx(7, 1));
            vertex_idx(4, 1) = min(id_y+2, vertex_idx(4, 1));
        }
        else
            vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + 1;
        
        // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
        // ############################################################################################################
        collide = false;
        for(id_y = vertex_idx(1, 1); id_y <= y_up; id_y++ )
        {   
            if( collide == true) 
                break;
            
            for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x-- )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(1, 2); id_z >= vertex_idx(5, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(1, 1) = max(id_y-2, vertex_idx(1, 1));
            vertex_idx(2, 1) = max(id_y-2, vertex_idx(2, 1));
            vertex_idx(6, 1) = max(id_y-2, vertex_idx(6, 1));
            vertex_idx(5, 1) = max(id_y-2, vertex_idx(5, 1));
        }
        else
            vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - 1;

        // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
        // ############################################################################################################
        int x_lo = max(0, vertex_idx(3, 0) - _step_length);
        int x_up = min(_max_x_id, vertex_idx(0, 0) + _step_length);

        collide = false;
        for(id_x = vertex_idx(0, 0); id_x <= x_up; id_x++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 0) = max(id_x-2, vertex_idx(0, 0)); 
            vertex_idx(1, 0) = max(id_x-2, vertex_idx(1, 0)); 
            vertex_idx(5, 0) = max(id_x-2, vertex_idx(5, 0)); 
            vertex_idx(4, 0) = max(id_x-2, vertex_idx(4, 0)); 
        }
        else
            vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - 1;    

        // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_x = vertex_idx(3, 0); id_x >= x_lo; id_x-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(3, 2); id_z >= vertex_idx(7, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(3, 0) = min(id_x+2, vertex_idx(3, 0)); 
            vertex_idx(2, 0) = min(id_x+2, vertex_idx(2, 0)); 
            vertex_idx(6, 0) = min(id_x+2, vertex_idx(6, 0)); 
            vertex_idx(7, 0) = min(id_x+2, vertex_idx(7, 0)); 
        }
        else
            vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + 1;

        // Z+ now is the above side : (p1 -- p2 -- p3 -- p4) face
        // ############################################################################################################
        collide = false;
        int z_lo = max(0, vertex_idx(4, 2) - _step_length);
        int z_up = min(_max_z_id, vertex_idx(0, 2) + _step_length);
        for(id_z = vertex_idx(0, 2); id_z <= z_up; id_z++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 2) = max(id_z-2, vertex_idx(0, 2));
            vertex_idx(1, 2) = max(id_z-2, vertex_idx(1, 2));
            vertex_idx(2, 2) = max(id_z-2, vertex_idx(2, 2));
            vertex_idx(3, 2) = max(id_z-2, vertex_idx(3, 2));
        }
        vertex_idx(0, 2) = vertex_idx(1, 2) = vertex_idx(2, 2) = vertex_idx(3, 2) = id_z - 1;

        // Z- now is the below side : (p5 -- p6 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_z = vertex_idx(4, 2); id_z >= z_lo; id_z-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(4, 1); id_y <= vertex_idx(5, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_x = vertex_idx(4, 0); id_x >= vertex_idx(7, 0); id_x-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(4, 2) = min(id_z+2, vertex_idx(4, 2));
            vertex_idx(5, 2) = min(id_z+2, vertex_idx(5, 2));
            vertex_idx(6, 2) = min(id_z+2, vertex_idx(6, 2));
            vertex_idx(7, 2) = min(id_z+2, vertex_idx(7, 2));
        }
        else
            vertex_idx(4, 2) = vertex_idx(5, 2) = vertex_idx(6, 2) = vertex_idx(7, 2) = id_z + 1;

        if(vertex_idx_lst == vertex_idx)
            break;

        vertex_idx_lst = vertex_idx;

        MatrixXd vertex_coord(8, 3);
        for(int i = 0; i < 8; i++)
        {   
            int index_x = max(min(vertex_idx(i, 0), _max_x_id - 1), 0);
            int index_y = max(min(vertex_idx(i, 1), _max_y_id - 1), 0);
            int index_z = max(min(vertex_idx(i, 2), _max_z_id - 1), 0);

            Vector3i index(index_x, index_y, index_z);
            Vector3d pos = collision_map->GridIndexToLocation(index);
            vertex_coord.row(i) = pos;
        }

        cubeMax.setVertex(vertex_coord, _resolution);
        if( isContains(lstcube, cubeMax))        
            return make_pair(lstcube, false); // 和上一个cube是包含关系

        iter ++;
    }

    return make_pair(cubeMax, true);
}

std::vector<Cube> CorridorGenerate::corridorGeneration(const vector<Vector3d> & path_coord, const vector<double> & time, const sdf_tools::CollisionMapGrid * collision_map)
{   
    vector<Cube> cubeList;
    Vector3d pt;

    Cube lstcube;
    int bound = 0;
    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        pt = path_coord[i];
        
        Cube cube = generateCube(pt, collision_map);
        auto result = inflateCube(cube, lstcube, collision_map);
        if(result.second == false)
            continue;

        cube = result.first;
        lstcube = cube;
        cube.t = time[i];

        if(i && (pt - path_coord[0]).norm() > _horizon)
            bound ++;
        if(bound == (int)(1.0/_resolution))
        {
            i = (int)path_coord.size() - 1;
            Vector3d endPt = path_coord[i];
            cube.box[0] = std::make_pair( min(cube.vertex(6, 0), endPt(0)), max(cube.vertex(0, 0), endPt(0)) );
            cube.box[1] = std::make_pair( min(cube.vertex(0, 1), endPt(1)), max(cube.vertex(1, 1), endPt(1)) );
            cube.box[2] = std::make_pair( min(cube.vertex(6, 2), endPt(2)), max(cube.vertex(1, 2), endPt(2)) );
            cube.t += 100.0;
        }

        cubeList.push_back(cube);
    }
    return cubeList;
    
    /*
    vector<Cube> cubecSimplifyList;
    for(auto cube:cubeList)
        if(cube.valid == true)
            cubecSimplifyList.push_back(cube);

    cubeList.clear();
    corridorSimplify(cubecSimplifyList, collision_map);
    return cubecSimplifyList;
    */
}

std::vector<Cube> CorridorGenerate::corridorGeneration(const vector<Vector3d> & path_coord, const sdf_tools::CollisionMapGrid * collision_map)
{   
    vector<Cube> cubeList;
    Vector3d pt;

    Cube lstcube;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt, collision_map);
        auto result = inflateCube(cube, lstcube, collision_map);

        if(result.second == false)
            continue;

        cube = result.first;
        
        lstcube = cube;
        cubeList.push_back(cube);
    }
    return cubeList;
    
    /*
    vector<Cube> cubecSimplifyList;
    for(auto cube:cubeList)
        if(cube.valid == true)
            cubecSimplifyList.push_back(cube);

    cubeList.clear();
    corridorSimplify(cubecSimplifyList, collision_map);
    return cubecSimplifyList;
    */
}

void CorridorGenerate::corridorSimplify(vector<Cube> & cubecList, const sdf_tools::CollisionMapGrid * collision_map)
{
    double Lg = 2.0;
    for(int i = 0; i < (int)cubecList.size() - 1; i ++ )
    {
        Cube &cube1 = cubecList[i];
        Cube &cube2 = cubecList[i+1];
        // printf("\n[corridorSimplify]: \n");
        // printf("cube%d:            x+=%f, x-=%f, y+=%f, y-=%f, z+=%f, z-=%f\n", i, cube1.vertex(0, 0), cube1.vertex(6, 0),cube1.vertex(6, 1), cube1.vertex(0, 1), cube1.vertex(0, 2), cube1.vertex(6, 2) );
        // printf("cube%d:            x+=%f, x-=%f, y+=%f, y-=%f, z+=%f, z-=%f\n", i+1, cube2.vertex(0, 0), cube2.vertex(6, 0),cube2.vertex(6, 1), cube2.vertex(0, 1), cube2.vertex(0, 2), cube2.vertex(6, 2) );
        if(cube1.vertex(6, 0) <= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(0, 2) &&
           cube2.vertex(6, 0) <= cube1.vertex(0, 0) && cube2.vertex(0, 1) <= cube1.vertex(6, 1) && cube2.vertex(6, 2) <= cube1.vertex(0, 2)  )
        {
            // reset the front cube +端
            {
                double xup = max(min(cube1.center(0) + Lg, cube1.vertex(0, 0)),       0.5*(cube1.vertex(0, 0) + cube2.vertex(6, 0)) );
                double yup = max(min(cube1.center(1) + Lg, cube1.vertex(6, 1)),       0.5*(cube1.vertex(6, 1) + cube2.vertex(0, 1)) );
                double zup = cube1.vertex(0, 2);
                // max(min(cube1.center(2) + Lg, cube1.vertex(0, 2)),       0.5*(cube1.vertex(0, 2) + cube2.vertex(6, 2)) );
                Vector3i up1_index = collision_map->LocationToGridIndex(Vector3d(xup, yup, zup));    
                Vector3d up1_coord = collision_map->GridIndexToLocation(up1_index);
                
                double x1_u = up1_coord(0);
                double y1_u = up1_coord(1);
                double z1_u = up1_coord(2);
                double x1_l = cube1.vertex(6, 0);
                double y1_l = cube1.vertex(0, 1);
                double z1_l = cube1.vertex(6, 2);
                if(i == 0)
                {
                    x1_u = cube1.vertex(0, 0);
                    y1_u *= 0.6;
                    // y1_u = cube1.vertex(6, 1);
                    if(cube2.center(1) > cube1.center(1))
                    {
                        y1_u *= 0.5;
                        x1_l = max(cube1.center(0) - Lg, cube1.vertex(6, 0));
                        y1_l = max(cube1.center(1) - Lg, cube1.vertex(0, 1));
                    }
                }
                cube1.vertex.row(0) = Vector3d(x1_u, y1_l, z1_u);  
                cube1.vertex.row(1) = Vector3d(x1_u, y1_u, z1_u);  
                cube1.vertex.row(2) = Vector3d(x1_l, y1_u, z1_u);  
                cube1.vertex.row(3) = Vector3d(x1_l, y1_l, z1_u);  

                cube1.vertex.row(4) = Vector3d(x1_u, y1_l, z1_l);  
                cube1.vertex.row(5) = Vector3d(x1_u, y1_u, z1_l);  
                cube1.vertex.row(6) = Vector3d(x1_l, y1_u, z1_l);  
                cube1.vertex.row(7) = Vector3d(x1_l, y1_l, z1_l);

                cube1.box[0] = std::make_pair( x1_l, x1_u );
                cube1.box[1] = std::make_pair( y1_l, y1_u );
                cube1.box[2] = std::make_pair( z1_l, z1_u );
            }

            // reset the next cube -端
            {
                double xlo = min(max(cube2.center(0) - Lg, cube2.vertex(6, 0)),       0.5*(cube1.vertex(0, 0) + cube2.vertex(6, 0)) ); 
                double ylo = min(max(cube2.center(1) - Lg, cube2.vertex(0, 1)),       0.5*(cube1.vertex(6, 1) + cube2.vertex(0, 1)) ); 
                double zlo = cube2.vertex(6, 2);
                // min(max(cube2.center(2) - Lg, cube2.vertex(6, 2)),       0.5*(cube1.vertex(0, 2) + cube2.vertex(6, 2)) ); 
                Vector3i lo2_index = collision_map->LocationToGridIndex(Vector3d(xlo, ylo, zlo));    
                Vector3d lo2_coord = collision_map->GridIndexToLocation(lo2_index);

                double x2_l = lo2_coord(0);
                double y2_l = lo2_coord(1);
                double z2_l = lo2_coord(2);
                double x2_u = cube2.vertex(0, 0);
                double y2_u = cube2.vertex(6, 1);
                double z2_u = cube2.vertex(0, 2);
                if(i && i + 1 == (int)cubecList.size()-1)
                {
                    //x2_u = min(cube2.vertex(0, 0), cube1.vertex(0, 0));
                    y2_u = 0.5*(cube2.vertex(6, 1) + cube1.vertex(6, 1));
                    if(cube2.center(1) > cube1.center(1))
                    {
                        y2_u = min(cube2.vertex(6, 1), cube1.vertex(6, 1));
                    }
                }
                cube2.vertex.row(0) = Vector3d(x2_u, y2_l, z2_u);  
                cube2.vertex.row(1) = Vector3d(x2_u, y2_u, z2_u);  
                cube2.vertex.row(2) = Vector3d(x2_l, y2_u, z2_u);  
                cube2.vertex.row(3) = Vector3d(x2_l, y2_l, z2_u);  

                cube2.vertex.row(4) = Vector3d(x2_u, y2_l, z2_l);  
                cube2.vertex.row(5) = Vector3d(x2_u, y2_u, z2_l);  
                cube2.vertex.row(6) = Vector3d(x2_l, y2_u, z2_l);  
                cube2.vertex.row(7) = Vector3d(x2_l, y2_l, z2_l);

                cube2.box[0] = std::make_pair( x2_l, x2_u );
                cube2.box[1] = std::make_pair( y2_l, y2_u );
                cube2.box[2] = std::make_pair( z2_l, z2_u );
            }

            // printf("cube%d simpfy as: ", i);
            // printf(" x+=%f, x-=%f, y+=%f, y-=%f, z+=%f, z-=%f\n", cube1.vertex(0, 0), cube1.vertex(6, 0),cube1.vertex(6, 1), cube1.vertex(0, 1), cube1.vertex(0, 2), cube1.vertex(6, 2) );
            // printf("cube%d simpfy as: ", i+1);
            // printf(" x+=%f, x-=%f, y+=%f, y-=%f, z+=%f, z-=%f\n", cube2.vertex(0, 0), cube2.vertex(6, 0),cube2.vertex(6, 1), cube2.vertex(0, 1), cube2.vertex(0, 2), cube2.vertex(6, 2) );
        
        }

    }

}

bool CorridorGenerate::generatePath(const vector<Cube> & cubecList, MatrixXd& local_path)
{
    
    vector<Vector3d> new_path;
    new_path.push_back(cubecList[0].center);

    for(int i = 1; i < (int)cubecList.size(); i ++ )
    {
        Cube cube1 = cubecList[i-1];
        Cube cube2 = cubecList[i];

        if(cube1.vertex(6, 0) <= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(0, 2) &&
           cube2.vertex(6, 0) <= cube1.vertex(0, 0) && cube2.vertex(0, 1) <= cube1.vertex(6, 1) && cube2.vertex(6, 2) <= cube1.vertex(0, 2)  )
        {
            double xup = min(cube1.vertex(0, 0), cube2.vertex(0, 0));
            double xlo = max(cube1.vertex(6, 0), cube2.vertex(6, 0));
            double yup = min(cube1.vertex(6, 1), cube2.vertex(6, 1));
            double ylo = max(cube1.vertex(0, 1), cube2.vertex(0, 1));        
            double zup = min(cube1.vertex(0, 2), cube2.vertex(0, 2));
            double zlo = max(cube1.vertex(6, 2), cube2.vertex(6, 2));

            // Vector3i cent_index = collision_map->LocationToGridIndex(Vector3d(0.5*(xup+xlo), 0.5*(yup+ylo), 0.5*(zup+zlo)));    
            // Vector3d cent_cood = collision_map->GridIndexToLocation(cent_index);
            // new_path.push_back(cent_cood);
            new_path.push_back(Vector3d(0.5*(xup+xlo), 0.5*(yup+ylo), 0.5*(zup+zlo)));
        }
        else{
            return false;
        }
        double cx = 0.5*(cube2.vertex(0, 0), cube2.vertex(6, 0));
        double cy = 0.5*(cube2.vertex(6, 1), cube2.vertex(0, 1));
        double cz = 0.5*(cube2.vertex(0, 2), cube2.vertex(6, 2));
        // Vector3i cent_index = collision_map->LocationToGridIndex(Vector3d(cx, cy, cz));    
        // Vector3d cent_cood = collision_map->GridIndexToLocation(cent_index);
        // new_path.push_back(cent_cood);
        new_path.push_back(Vector3d(cx, cy, cz));
    }
    
    local_path = MatrixXd::Zero(3, (int)new_path.size());
    for(int k = 0; k < (int)new_path.size(); k ++)
    {
        local_path(0,k) = new_path[k](0);
        local_path(1,k) = new_path[k](1);
        local_path(2,k) = new_path[k](2);
    }
    return true;
}