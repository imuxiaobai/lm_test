#ifndef LM_TEST_PROBLEM_H
#define LM_TEST_PROBLEM_H

#include <unordered_map>
#include <map>
// #include <memory>

#include "edge.hpp"
#include "vertex.hpp"
#include "vertex_pose.hpp"
#include "vertex_inverse_depth.hpp"
#include "types.hpp"

namespace lm_test{

class Problem{
    /*先假设是普通最小二乘问题 */
public:
    enum class ProblemType {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };
    typedef std::map<Id_type, std::shared_ptr<Vertex>> WholeVertex;
    typedef std::unordered_map<Id_type, std::shared_ptr<Edge>> WholeEdge;
    typedef std::unordered_multimap<Id_type, std::shared_ptr<Edge>> HashVertexIdToEdge;

    Problem(ProblemType problemType) : problemType_(problemType = ProblemType::GENERIC_PROBLEM){}
    ~Problem(){}

    /*关于新建vertex */
    bool addVertex(Vertex_type vertex){
        if(vertexs_.find(vertex->id()) != vertexs_.end()){
            printf("This %ld vertex has add.", vertex->id());
            return false;
        }
        else{
            vertexs_.insert(std::pair<Id_type, Vertex_type>(vertex->id(), vertex));
        }
        if(problemType_ == ProblemType::SLAM_PROBLEM){
            if(IsPoseVertex(vertex)){ 
                resizePoseHessionWhenAddingPose(vertex);
            }
        }
        return true;
    }

    bool IsPoseVertex(Vertex_type &v){
        std::string type = v->TypeInfo();
        return type == std::string("VertexPose");
    }
    bool IsLandmarkVertex(Vertex_type v){
        std::string type = v->TypeInfo();
        return type == std::string("VertexInverseDepth") || type == std::string("VertexPoint");
    }

    void resizePoseHessionWhenAddingPose(Vertex_type v) {   //如果新加入了状态变量，需要将先验部分矩阵扩展，之后再加到当前的H矩阵上
        int size = h_prior_.rows() + v->x_dimension();
        h_prior_.conservativeResize(size, size);    //扩展矩阵，同时旧的数据保留
        b_prior_.conservativeResize(size);

        b_prior_.tail(v->local_dimension()).setZero();
        h_prior_.rightCols(v->local_dimension()).setZero();
        h_prior_.bottomRows(v->local_dimension()).setZero();
    }

    /*todo 删掉vertex后，将对应的边也去掉*/
    bool removeVertex(Vertex_type vertex){
        if(vertexs_.find(vertex->id()) == vertexs_.end()){
            return false;
        }
        vector<Edge_type> remove_edge = getConnectEdges(vertex);
        for(uint i = 0; i < remove_edge.size(); i++){
            removeEdge(remove_edge[i]);
        }
        // vertex->setOrderingId(-1);
        vertexToEdge_.erase(vertex->id());
        vertexs_.erase(vertex->id());
        if(IsPoseVertex(vertex)){
            idx_pose_vertexs.erase(vertex->id());
        }
        else if(IsLandmarkVertex(vertex)){
            idx_landmark_vertexs.erase(vertex->id());
        }
    }

    /*用于删除节点时找到对应的边 */
    vector<Edge_type> getConnectEdges(Vertex_type v){
        vector<Edge_type> edges;
        auto range = vertexToEdge_.equal_range(v->id()); //通过vertex的id 找到对应的edge范围
        for(auto iter = range.first; iter != range.second; iter++) {    //每一个iter指的是一个map的指针,first是vertex的id，second是连接的edge
            if(edges_.find(iter->second->id()) == edges_.end()){
                continue;
            }
            edges.emplace_back(iter->second);
        }
        return edges;
    }

    /*关于新建edge */
    bool addEdge(Edge_type edge){
        if(edges_.find(edge->id()) != edges_.end()){
            printf("This %ld edge has add.", edge->id());
            return false;
        }
        else{
            edges_.insert(std::pair<Id_type, Edge_type>(edge->id(), edge));
            // std::cout << "edges_size" << edges_.size() << std::endl;
        }
        // todo
        for (auto &vertex: edge->vertexs()) {
            vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->id(), edge));
        }
        return true;
    }

    bool removeEdge(Edge_type edge){
        if(edges_.find(edge->id()) == edges_.end()) {
            return false;
        }
        edges_.erase(edge->id());
        return true;
    }

    //todo 20190722
    bool solveProblem(){
        if (edges_.size() == 0 || vertexs_.size() == 0) {
            std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
            return false;
        }
        setOrdering();
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

    /*搞定了ordering */
    void setOrdering(){
        /*pose和landmarks的维数 */
        ordering_poses_ = 0;
        ordering_landmarks_ = 0;
        ordering_generic_ = 0;//普通问题总维数

        for(auto &vertex: vertexs_) {   //注意此时的vertex是包含了状态id的map
            ordering_generic_ += vertex.second->local_dimension();
            // if(IsPoseVertex(vertex.second))  {
            // }
            if(problemType_ == ProblemType::SLAM_PROBLEM){
                addOrderingSLAM(vertex.second);
            }
        }
    }

    /*每一次计算H矩阵之前，将现存所有的vertex 根据id 排列到对应的pos或landmark里 并将其在H矩阵中对应的位置set上 */
    void addOrderingSLAM(Vertex_type v){
        if (IsPoseVertex(v)) {
            v->setOrderingId(ordering_poses_);
            ordering_poses_ += v->local_dimension();
            idx_pose_vertexs.insert(pair<ulong, Vertex_type>(v->id(), v));
        }
        else if (IsLandmarkVertex(v)){
            v->setOrderingId(ordering_landmarks_);
            ordering_landmarks_ += v->local_dimension();
            idx_landmark_vertexs.insert(pair<ulong, Vertex_type>(v->id(), v));
        }
    }

    void setHession(){
        /*遍历每个节点，求解总的状态大小，得到Hession矩阵维度 */
        // whole_dimension = 0;
        // for(std::pair<Id_type, Vertex_type> vertex : vertexs_){
        //     whole_dimension += vertex.second->x_dimension();

        //     // std::cout << "whole_dimension" << whole_dimension << std::endl;
        // }
        // hession_.resize(whole_dimension, whole_dimension);
        // b_.resize(whole_dimension);
        hession_.resize(ordering_generic_, ordering_generic_);
        b_.resize(ordering_generic_);
        hession_.setZero();
        b_.setZero();

        /*遍历每个边，计算其雅各比矩阵，并加到一起得到H矩阵 */
        for(auto &edge: edges_){
            edge.second->computeJacobian();
            edge.second->computeResidual(0);

            Jacobians_type jacobians = edge.second->jacobians();
            Residual_type residual = edge.second->residual();
            std::vector<Vertex_type> vertexs = edge.second->vertexs();
            assert(jacobians.size() == vertexs.size()); //todo

            // Id_type ordering_id_i = 0;
            
            // std::cout << "test5 :" << ordering_id_i << std::endl;
            /*殘差对”每一个“状态变量x的雅各比矩阵 例: dr/dp dr/dv dr/dq*/
            for(Id_type i = 0; i < vertexs.size(); i++){
                if(vertexs[i]->is_fixed())  continue;

                Per_Jacobian_type per_jacobian_i = jacobians[i];
                Information_type inform = edge.second->information();
                // Dimension_type x_dimension_i = vertexs[i]->x_dimension(); 
                Dimension_type x_dimension_i = vertexs[i]->local_dimension(); 
                Id_type ordering_id_i = vertexs[i]->ordering_id();
                // Id_type ordering_id_j = 0;

                for(Id_type j = i; j < vertexs.size(); j++){
                    if(vertexs[j]->is_fixed()) continue;

                    Per_Jacobian_type per_jacobian_j = jacobians[j];
                    // Dimension_type x_dimension_j = vertexs[j]->x_dimension(); 
                    Dimension_type x_dimension_j = vertexs[j]->local_dimension(); 
                    Id_type ordering_id_j = vertexs[j]->ordering_id();

                    Hession_type hession = per_jacobian_i.transpose() * inform * per_jacobian_j;
                    // std::cout << "per_jacobian_i:" << per_jacobian_i << std::endl;
                    // std::cout << "inform:" << inform << std::endl;
                    // std::cout << "per_jacobian_j:" << per_jacobian_j << std::endl;
                    hession_.block(ordering_id_i, ordering_id_j, x_dimension_i, x_dimension_j).noalias() += hession;//noalias()说明矩阵没有混淆，减少临时变量
                    /*H是对称的，下三角和上三角是对应的 */
                    if(j != i)
                        {hession_.block(ordering_id_j, ordering_id_i, x_dimension_j, x_dimension_i).noalias() += hession.transpose();} 

                    // ordering_id_j += x_dimension_j;
                }
                b_.segment(ordering_id_i, x_dimension_i).noalias() -= per_jacobian_i.transpose() * inform * residual;
                // ordering_id_i += x_dimension_i;
            }
        }
        /*todo 先验 如果存在fix 的点，将其从先验信息中排除？？？*/
        if(h_prior_.rows() > 0){
            Hession_type h_prior_tmp = h_prior_;
            B_type b_prior_tmp = b_prior_;
            for(auto v : vertexs_){
                if(IsPoseVertex(v.second) && (v.second->is_fixed())){
                    int idx = v.second->id();
                    int dim = v.second->x_dimension();
                    h_prior_tmp.block(0, idx, h_prior_.rows(), dim).setZero();
                    h_prior_tmp.block(idx, 0, dim, h_prior_.rows()).setZero();
                    b_prior_tmp.segment(idx, dim).setZero();
                }
            }
            // if(err_prior.rows() > 0){   //重要！先验部分随着delta_x的更新，更新b
            //     b_prior_ -= h_prior_ * delta_x_.head(ordering_poses_);
            // }
            hession_.topLeftCorner(ordering_poses_, ordering_poses_) += h_prior_tmp;
            b_.head(ordering_poses_) += b_prior_tmp;   //只有pose部分存在先验 
        }

        // std::cout << "H:" << hession_ << std::endl;
        delta_x_ = VectorXd::Zero(ordering_generic_);//todo 重新初始化 delta_x
    }

    void computeErrorInit(){
        for(auto &edge: edges_){
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
        for(Dimension_type i = 0; i < unsigned(hession_.cols()); i++){
            maxDiagonal = std::max(fabs(hession_(i, i)), maxDiagonal);
        }
        u_ = tau * maxDiagonal;
    }

    void solveLM(){
        /*计算delta_x */
        if(problemType_ == ProblemType::GENERIC_PROBLEM)
        {
            Hession_type hession_tmp = hession_;
            for(Dimension_type i = 0; i < unsigned(hession_tmp.cols()); i++){
                hession_tmp(i, i) = hession_tmp(i, i) + u_;
            }
            /*A*x=b 得到delta_x_*/
            // delta_x_ = hession_tmp.inverse() * b_;
            delta_x_ = hession_tmp.ldlt().solve(b_);   //todo
        }
        else if(problemType_ == ProblemType::SLAM_PROBLEM)
        {   //schur
            // int reserve_size = ordering_poses_;
            int reserve_size = ordering_poses_;
            int marg_size = ordering_landmarks_;
            //将每一部分提出来 p--pose m--landmark //需要一次疏尔补
            Hession_type Hmm = hession_.block(reserve_size, reserve_size, marg_size, marg_size);
            Hession_type Hpm = hession_.block(0, reserve_size, reserve_size, marg_size);
            Hession_type Hmp = hession_.block(reserve_size, 0, marg_size, reserve_size);
            B_type bpp = b_.segment(0, reserve_size);
            B_type bmm = b_.segment(reserve_size, marg_size);

            Hession_type Hmm_inv(Hession_type::Zero(marg_size, marg_size));
            for(auto landmarkVertex : idx_landmark_vertexs){
                int idx = landmarkVertex.second->id();
                int dim = landmarkVertex.second->local_dimension();
                Hmm_inv.block(idx, idx, dim, dim) = Hmm_inv.block(idx, idx, dim, dim).reverse();
            }
            H_pp_schur_ = hession_.block(0, 0, ordering_poses_, ordering_poses_) - Hpm * Hmm_inv * Hmp;
            b_pp_schur_ = bmm - Hpm * Hmm_inv * bpp;
            for(Id_type i = 0; i < ordering_poses_; i++){
                H_pp_schur_(i, i) += u_;
            }
            //先计算pos部分
            State_type delta_x_pp(State_type::Zero(reserve_size));
            delta_x_pp = H_pp_schur_.ldlt().solve(b_pp_schur_);
            //再计算landmark部分
            State_type delta_x_mm(State_type::Zero(marg_size));
            delta_x_mm = Hmm_inv * (bmm - Hmp * delta_x_pp);
            //加到整体状态变量上
            delta_x_.head(reserve_size) = delta_x_pp;
            delta_x_.tail(marg_size) = delta_x_mm;
        }
    }

    bool updateState(){
        // bool updateSuccess = false;
        /*尝试更新状态x */
        for(auto &vertex: vertexs_){
            Id_type id = vertex.second->ordering_id();
            Dimension_type dim = vertex.second->x_dimension();
            State_type delta_x_update = delta_x_.segment(id, dim);
            vertex.second->update_tmp(delta_x_update);
        }
        if(err_prior_.rows() > 0){
            b_prior_back = b_prior_;
            err_prior_back = err_prior_;
            b_prior_ -= h_prior_ * delta_x_.head(ordering_poses_);
            err_prior_ = -Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 15);
        }
        
        // updateSuccess = updateSuccess();
        /*如果成功更新，就将更新应用到状态x上 */
        if(updateSuccess()){
            for(auto &vertex: vertexs_){
                Id_type id = vertex.second->ordering_id();
                Dimension_type dim = vertex.second->x_dimension();
                State_type delta_x_update = delta_x_.segment(id, dim);
                vertex.second->update();
            }
            return true;
        } else {
            if(err_prior_.rows() > 0){
                b_prior_ = b_prior_back;
                err_prior_ = err_prior_back;
            }
            return false;
        }
    }

    bool updateSuccess(){//ok
        /*计算残差，看是否下降了 */
        double error_tmp = 0;
        for(auto &edge: edges_){
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

    /*todo PCG */
    State_type PCGSolver(const Hession_type &A, const B_type &b, int maxIter = -1) {
        assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
        int rows = b.rows();
        int n = maxIter < 0 ? rows : maxIter;
        State_type x(VectorXd::Zero(rows));
        Hession_type M_inv = A.diagonal().asDiagonal().inverse();
        B_type r0(b);  // initial r = b - A*0 = b
        B_type z0 = M_inv * r0;
        B_type p(z0);
        B_type w = A * p;
        double r0z0 = r0.dot(z0);
        double alpha = r0z0 / p.dot(w);
        B_type r1 = r0 - alpha * w;
        int i = 0;
        double threshold = 1e-6 * r0.norm();
        while (r1.norm() > threshold && i < n) {
            i++;
            B_type z1 = M_inv * r1;
            double r1z1 = r1.dot(z1);
            double belta = r1z1 / r0z0;
            z0 = z1;
            r0z0 = r1z1;
            r0 = r1;
            p = belta * p + z1;
            w = A * p;
            alpha = r1z1 / p.dot(w);
            x += alpha * p;
            r1 -= alpha * w;
        }
        return x;
    }
    /*todo marg 某个节点，将对应的imu和特征点也删除*/
    /*输入为要边缘化的节点 */
    bool marginalize(const std::vector<Vertex_type> margVertexs, int pose_dim){
        std::vector<Edge_type> marg_edges = getConnectEdges(margVertexs[0]); //只marg第一帧
        std::unordered_map<int, Vertex_type> margLandmark;
        int marg_landmark_size = 0;
        for(size_t i = 0; i < marg_edges.size(); i++){
            auto vertexs = marg_edges[i]->vertexs();
            for(auto v : vertexs){
                if(IsLandmarkVertex(v) && (margLandmark.find(v->id()) == margLandmark.end())){
                    v->setOrderingId(pose_dim + marg_landmark_size);
                    margLandmark.insert(std::pair(v->id(), v));
                    marg_landmark_size += v->local_dimension();
                }
            }
        }
        


    }

private:
    ProblemType problemType_;

    /*整体信息矩阵 及 状态变量*/
    Hession_type hession_;
    State_type delta_x_;
    B_type b_;

    /*先验部分，先用疏尔补把要边缘化的部分marg掉，剩下的部分直接加在下一时刻的 H 矩阵上 */
    /*注意  此时的EFJ，即边缘化掉的特征对应的雅各比矩阵线性化点是不变的 */
    Hession_type h_prior_;
    B_type b_prior_;
    Per_Jacobian_type Jt_prior_inv_;    //求解ceres时把先验变换成雅各比和殘差形式
    Residual_type err_prior_;
    //x的每次更新，b的先验 即 err的先验会随着变化，这是由于
    //先验信息  todo
    B_type b_prior_back;
    Residual_type err_prior_back;

    /*H矩阵的 pose部分 和 landmark部分，用于分块疏尔补求解（但是这里后来 这几项直接写在函数里面了，就是Hmm,Hpm,Hmp） */
    // Hession_type h_pp;
    // Hession_type h_ll;
    // B_type b_pp;
    // B_type b_ll;
    /*求解x的时候先计算出 pos ，再计算出landmark ，所以要将H 矩阵分块*/
    Hession_type H_pp_schur_;
    B_type b_pp_schur_;

    /*所有的边与点 */
    WholeEdge edges_;
    WholeVertex vertexs_;
    HashVertexIdToEdge vertexToEdge_;   //通过vertex 找到edge  所以前面的id是vertex的id, 后面的vector存和vertex相连的edge 用在删vertex时链接的edge
    // WholeDimension whole_dimension;    //H矩阵的维度，被下面的ordering_generic_代替了

    //维度
    Dimension_type ordering_poses_ = 0;
    Dimension_type ordering_landmarks_ = 0;
    Dimension_type ordering_generic_ = 0;   //普通问题的维度，也可以作为整个H 矩阵的维度

    /*求解LM用的参数 */
    double error = 0;

    double u_ = 0.0; //即LM的u
    double v_ = 2.0; //更新u用的
    double tau = 1e-5; //求解u初始值的那个参数tau

    int max_iter = 100; //最大迭代次数
    double delta_x_threshold = 1e-6;
    double error_threshold = 1e-6;

    
    /*以ordering排序的顶点 */
    WholeVertex idx_pose_vertexs;
    WholeVertex idx_landmark_vertexs;
};

}

#endif
