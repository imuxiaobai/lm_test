#ifndef LM_TEST_VERTEX_H
#define LM_TEST_VERTEX_H

#include "types.hpp"
using namespace Eigen;
namespace lm_test{

static Id_type vertex_id = 0;
class Vertex{
public:
    explicit Vertex(Dimension_type x_dimension_):x_dimension_(x_dimension_){
        x_.resize(x_dimension_);
        x_tmp_.resize(x_dimension_);
        vertex_id_ = vertex_id++;
    }

    virtual ~Vertex(){}
    
    /*状态变量临时更新 */
    /*若状态变量回滚，delta_x_加上后，error变大了，所以不要这一次的迭代结果 */
    void update_tmp(State_type delta_x_){
        x_tmp_ = x_ + delta_x_;
    }
    void update(){
        x_ = x_tmp_;
    }


    /* 返回状态变量 */
    State_type x() const {
        return x_;
    }
    State_type x_tmp() const {
        return x_tmp_;
    }
    /* 返回维数,id,oid */
    Dimension_type x_dimension() const {
        // std::cout << "  " << x_dimension_ << std::endl;
        return x_dimension_;
    }
    Id_type id() const {
        return vertex_id_;
    }
    Id_type ordering_id() const {
        return ordering_id_;
    }
    bool is_fixed() const {
        return fixed_;
    }

    void setState(const State_type &state){
        x_ = state;
    }
    void setOrderingId(const Id_type &id){
        ordering_id_ = id;
    }
    void setFixed(bool fixed = true){
        fixed_ = fixed;
    }

protected:
    /* 顶点维度 */
    State_type x_;
    State_type x_tmp_;

    /* 状态量维度， 如位姿是6或7维 */
    Dimension_type x_dimension_;

    /* 状态量的id,每新建一个就自动累加 */
    Id_type vertex_id_ = 0;
    // vectorXd vertex;

    /*  ordering id是在problem中排序后的id，用于寻找雅可比对应块
     ordering id带有维度信息，例如ordering_id=6则对应Hessian中的第6列(再加上x_dimension_就是该状态矩阵在H中的最后一行了)
     从零开始 */
    Id_type ordering_id_ = 0;

    bool fixed_ = false;  //该状态变量是否被固定
};
typedef std::shared_ptr<Vertex> Vertex_type;
}


#endif
