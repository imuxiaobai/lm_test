#ifndef LM_TEST_PROBLEM_H
#define LM_TEST_PROBLEM_H

#include <unordered_map>
#include <map>
// #include <memory>

#include "edge.hpp"
#include "vertex.hpp"
#include "types.hpp"

namespace lm_test{

class Problem{
    /*先假设是普通最小二乘问题 */
public:
    typedef std::map<Id_type, std::shared_ptr<Vertex>> WholeVertex;
    typedef std::unordered_map<Id_type, std::shared_ptr<Edge>> WholeEdge;
    typedef std::unordered_multimap<Id_type, std::shared_ptr<Edge>> HashVertexIdToEdge;

    Problem(){}
    ~Problem(){}

    bool addVertex(Vertex_type vertex){
        if(vertexs_.find(vertex->id()) != vertexs_.end()){
            printf("This %ld vertex has add.", vertex->id());
            return false;
        }
        else{
            vertexs_.insert(std::pair<Id_type, Vertex_type>(vertex->id(), vertex));
            return true;
        }
    }

    // bool removeVertex(Vertex_type vertex){;}

    bool addEdge(Edge_type edge){
        if(edges_.find(edge->id()) != edges_.end()){
            printf("This %ld edge has add.", edge->id());
            return false;
        }
        else{
            edges_.insert(std::pair<Id_type, Edge_type>(edge->id(), edge));
            // std::cout << "edges_size" << edges_.size() << std::endl;
            
        }

        return true;
    }

    // bool removeEdge(Edge_type edge){;}

    //todo 20190722
    bool solveProblem(){
        if (edges_.size() == 0 || vertexs_.size() == 0) {
            std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
            return false;
        }
        // std::cout << "test0" << std::endl;
        setHession();
        // std::cout << "test1" << std::endl;
        computeErrorInit();
        // std::cout << "test2" << std::endl;
        computeU();
        // std::cout << "test3" << std::endl;
        /*迭代求解, 直到到达最大迭代次数，或遇到情况，用stop_flag彻底停止迭代*/
        /*其他停止迭代方法：1.十次选取u，都没使error减少  2.delta_x很小  3.error过小  4.b_max过小(这里没写) */
        bool stop_flag = false;
        int iter = 0;
        while((!stop_flag) && (iter < max_iter)){
            int false_cnt = 0;
            /*不断尝试更改u和v，使得error减少一次 *//*如果十次都没减少,即false_cnt > 10，用stop_flag彻底退出迭代 */
            bool updateSuccess = false;
            while(!updateSuccess){
                /* 1. 十次更新u，都没使error减少  */
                if(false_cnt > 10){
                    stop_flag = true;
                    break;
                }

                /*求解delta_x */
                solveLM();

                /*2. delta_x 过小，直接退出迭代  squaredNorm()是二范数 */
                if(delta_x_.squaredNorm() <= delta_x_threshold){
                    stop_flag = true;
                    break;
                }

                /*判断error是否减小了，是的话进行更新，否则认为失败，更新u和v重新计算 */
                updateSuccess =  updateState();
                if(updateSuccess){
                    false_cnt = 0;
                    setHession();   //todo  需要重新选取线性化点吗？
                } else {
                    false_cnt++;
                }
            }
            iter++;
            /*  3. error 跟第一次的error相比，下降了 1e6 倍则退出 */
            if (std::sqrt(error) < error_threshold)
                stop_flag = true;
        }
        return false;
    }

    void setHession(){
        /*遍历每个节点，求解总的状态大小，得到Hession矩阵维度 */
        whole_dimension = 0;
        for(std::pair<Id_type, Vertex_type> vertex : vertexs_){
            whole_dimension += vertex.second->x_dimension();

            // std::cout << "whole_dimension" << whole_dimension << std::endl;
        }
        hession_.resize(whole_dimension, whole_dimension);
        b_.resize(whole_dimension);
        hession_.setZero();
        b_.setZero();

        /*遍历每个边，计算其雅各比矩阵，并加到一起得到H矩阵 */
        for(std::pair<Id_type, Edge_type> edge: edges_){
            edge.second->computeJacobian();
            edge.second->computeResidual(0);

            Jacobians_type jacobians = edge.second->jacobians();
            Residual_type residual = edge.second->residual();
            std::vector<Vertex_type> vertexs = edge.second->vertexs();
            assert(jacobians.size() == vertexs.size()); //todo

            Id_type ordering_id_i = 0;
            
            // std::cout << "test5 :" << ordering_id_i << std::endl;
            /*殘差对”每一个“状态变量x的雅各比矩阵 例: dr/dp dr/dv dr/dq*/
            for(Id_type i = 0; i < vertexs.size(); i++){
                Per_Jacobian_type per_jacobian_i = jacobians[i];
                Information_type inform = edge.second->information();
                Dimension_type x_dimension_i = vertexs[i]->x_dimension(); 
                //// Id_type ordering_id_i = vertexs[i]->ordering_id();
                Id_type ordering_id_j = 0;
                for(Id_type j = i; j < vertexs.size(); j++){
                    Per_Jacobian_type per_jacobian_j = jacobians[j];
                    Dimension_type x_dimension_j = vertexs[j]->x_dimension(); 
                    // Id_type ordering_id_j = vertexs[j]->ordering_id();
                    Hession_type hession = per_jacobian_i.transpose() * inform * per_jacobian_j;
                    // std::cout << "per_jacobian_i:" << per_jacobian_i << std::endl;
                    // std::cout << "inform:" << inform << std::endl;
                    // std::cout << "per_jacobian_j:" << per_jacobian_j << std::endl;
                    hession_.block(ordering_id_i, ordering_id_j, x_dimension_i, x_dimension_j) += hession;
                    /*H是对称的，下三角和上三角是对应的 */
                    if(j != i)
                        {hession_.block(ordering_id_j, ordering_id_i, x_dimension_j, x_dimension_i) += hession.transpose();} 

                    ordering_id_j += x_dimension_j;
                }
                b_.segment(ordering_id_i, x_dimension_i).noalias() -= per_jacobian_i.transpose() * inform * residual;
                ordering_id_i += x_dimension_i;
            }

        }
        // std::cout << "H:" << hession_ << std::endl;
    }

    void computeErrorInit(){
        for(auto edge: edges_){
            error += edge.second->residual2Mahalanobis(edge.second->residual());
        }
        // std::cout << "error:" << error << std::endl;

        //todo prior
        // if (err_prior_.rows() > 0)
        // error += err_prior_.norm();

        error_threshold = 1e-6 * std::sqrt(error);          // 迭代条件为 误差下降 1e-6 倍
    }

    void computeU(){
        /*LM法计算lambda(即jTj+uI里的u) */
        double maxDiagonal = 0;
        for(Dimension_type i = 0; i < hession_.cols(); i++){
            maxDiagonal = std::max(fabs(hession_(i, i)), maxDiagonal);
        }
        u_ = tau * maxDiagonal;
    }

    void solveLM(){
        /*计算A */
        Hession_type hession_tmp = hession_;
        for(Dimension_type i = 0; i < hession_tmp.cols(); i++){
            hession_tmp(i, i) = hession_tmp(i, i) + u_;
        }
        /*A*x=b 得到delta_x_*/
        delta_x_ = hession_tmp.inverse() * b_;
        // delta_x_ = H.ldlt().solve(b_);   //todo
    }

    bool updateState(){
        // bool updateSuccess = false;
        /*尝试更新状态x */
        for(auto vertex: vertexs_){
            Id_type id = vertex.second->ordering_id();
            Dimension_type dim = vertex.second->x_dimension();
            State_type delta_x_update = delta_x_.segment(id, dim);
            vertex.second->update_tmp(delta_x_update);
        }
        // updateSuccess = updateSuccess();
        /*如果成功更新，就将更新应用到状态x上 */
        if(updateSuccess()){
            for(auto vertex: vertexs_){
                Id_type id = vertex.second->ordering_id();
                Dimension_type dim = vertex.second->x_dimension();
                State_type delta_x_update = delta_x_.segment(id, dim);
                vertex.second->update();
            }
            return true;
        } else {
            return false;
        }
    }

    bool updateSuccess(){//ok
        /*计算惨差，看是否下降了 */
        double error_tmp = 0;
        for(auto edge: edges_){
            edge.second->computeResidual(1);
            error_tmp += edge.second->residual2Mahalanobis(edge.second->residual());
        }
        
        /* 计算rho，用来更新u和v */
        double delta_L = 0.5 * delta_x_.transpose() * (u_ * delta_x_ + b_);
        // std::cout << "b:" << b_ << std::endl;
        // std::cout << "delta_x:" << delta_x_ << std::endl;
        delta_L += 1e-3; //确保其非0
        double rho = (error - error_tmp) / delta_L;
        std::cout << "error 0:" << error << std::endl;
        // std::cout << "error_tmp:" << error_tmp << std::endl;
        // std::cout << "delta_L:" << delta_L << std::endl;
        // std::cout << "rho" << rho << std::endl;
        if(rho > 0) {
            /*用rou更新lambda */
            double alpha = std::min(1. - pow((2 * rho - 1), 3), 2./3.);
            u_ = u_ * std::max(double(1./3.), alpha);
            v_ = 2.0;
            error = error_tmp;
            std::cout << "u:" << u_ << std::endl;
            // std::cout << "v:" << v_ << std::endl;
            // std::cout << std::endl;
            return true;
        }
        else {
            u_ *=  v_;
            v_ *= 2.0;
            // std::cout << "re_rho" << std::endl;
            return false;
        }
    }
private:
    Hession_type hession_;
    State_type delta_x_;
    B_type b_;

    WholeEdge edges_;
    WholeVertex vertexs_;

    WholeDimension whole_dimension;    

    double error = 0;

    double u_ = 0.0; //即LM的u
    double v_ = 2.0; //更新u用的
    double tau = 1e-5; //求解u初始值的那个参数tau

    int max_iter = 100; //最大迭代次数
    double delta_x_threshold = 1e-6;
    double error_threshold = 1e-6;
};

}

#endif
