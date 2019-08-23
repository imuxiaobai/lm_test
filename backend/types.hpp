#ifndef LM_TEST_TYPES_H_
#define LM_TEST_TYPES_H_
#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
#include <unordered_map>
#include <math.h>
// #include <vertex.h>
// #include <edge.h>

namespace lm_test{
    
//设y=Jx 其中 y维数为n×1, 雅各比矩阵J为 n*m, x维数为 m×1 [在这里x为abc，3×1  y为y，1×1]  
//HX=b H矩阵也为 m*m  , b矩阵为 m*1
// class Vertex;
// class Edge;
/*vertex */
typedef Eigen::VectorXd State_type; //x delta_x
typedef unsigned int Dimension_type;
typedef unsigned long int Id_type;

/*edge */
typedef std::vector<Eigen::MatrixXd> Jacobians_type;
typedef Eigen::MatrixXd Per_Jacobian_type;
typedef Eigen::VectorXd Residual_type;
typedef Eigen::MatrixXd Information_type;
typedef Eigen::VectorXd Observation_type;

/*problem */
// typedef std::map<Id_type, std::shared_ptr<Vertex>> WholeVertex;
// typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> WholeEdge;
// typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

// typedef Eigen::Matrix<double, -1, -1> Hession_type;
typedef Eigen::MatrixXd Hession_type;
typedef Eigen::VectorXd B_type;
typedef unsigned long int WholeDimension; 

}




#endif