#ifndef LM_TEST_EDGE_H
#define LM_TEST_EDGE_H

#include <memory>
#include "vertex.hpp"
#include "types.hpp"

namespace lm_test{
/*代表一条边 */
static Id_type edge_id = 0;
class Edge{
public:
    explicit Edge(Dimension_type residual_dimension){
        residual_.resize(residual_dimension);
        jacobians_.resize(residual_dimension);  //todo
        edge_id_ = edge_id++;

        Information_type information(residual_dimension, residual_dimension);
        information.setIdentity();
        information_ = information;
    }
    virtual ~Edge(){}

    /*为边添加一个节点 */
    bool addVertex(Vertex_type vertex){
        vertex_connect_.emplace_back(vertex);   //当数据为类时用emplace_back,防止额外复制操作。
        return true;
    }
    /*添加全部和边相连的节点 */
    bool addWholeVertexs(const std::vector<Vertex_type> &vertexs){
        vertex_connect_ = vertexs;
        return true;
    }

    /*殘差转马氏距离，求解error时用 */
    double residual2Mahalanobis(Residual_type residual_){
        double mahal_dis = residual_.transpose() * information_ * residual_;
        return mahal_dis;
    }

    /*在子类的时候具体再写，每个例子不一样 */
    virtual void computeResidual(int tmp_flag){};

    virtual void computeJacobian(){};

    void setInformation(const Information_type &information){
        information_ = information;
    }
    /*返回各个值，id、和边相连的点、雅各比矩阵、殘差、信息矩阵 */
    Id_type id() const {
        return edge_id_;
    }
    std::vector<Vertex_type> vertexs() const {
        return vertex_connect_;
    }
    Vertex_type vertex(int i) const {
        return vertex_connect_[i];
    }
    Residual_type residual() const {
        return residual_;
    } 
    Jacobians_type jacobians() const {
        return jacobians_;
    }
    Information_type information() const { 
        return information_;
    }
    Id_type vertex_size() const {
        return vertex_connect_.size();
    }

    Id_type OrderingId() const { return ordering_id_; } //todo

    void SetOrderingId(const int &id) { ordering_id_ = id; };
    
protected:
    Id_type edge_id_ = 0;   //边的id
    std::vector<Vertex_type> vertex_connect_;   //与此边连接的顶点
    Residual_type residual_;  //此边所代表的殘差 m*1
    Jacobians_type jacobians_;   //雅各比矩阵，表示  delta殘差/delta_x   vector中每一个代表殘差向量对每一个x特征向量的雅各比  每一个m*n的雅各比矩阵，m是殘差维度，n是特征维度
    Information_type information_;  //信息矩阵  殘差转马氏距离 m*m
    Observation_type observation_;  //观测矩阵  输出y  m*1  //todo

    Id_type ordering_id_;   //todo
};

typedef std::shared_ptr<Edge> Edge_type;
}

#endif
