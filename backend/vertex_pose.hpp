#ifndef VERTEX_POSE_H
#define VERTEX_POSE_H
#include "vertex.hpp"
using namespace Eigen;

namespace lm_test{

class VertexPose : public Vertex {
public:
    VertexPose() : Vertex(7) {}
    void plus(const State_type &delta_x) override {
        State_type x_;
        x_.resize(x_dimension());
        x_ = x();
        x_.head<3>() += delta_x.head<3>();
        Qd q(x_[6], x_[3], x_[4], x_[5]);
        Qd delta_q(delta_x[6], delta_x[3], delta_x[4], delta_x[5]);//实际储存为6维todo
        q = q * delta_q;
        q.normalized();
        x_[3] = q.x();
        x_[4] = q.y();
        x_[5] = q.z();
        x_[6] = q.w();
        setState(x_);
    }

    std::string TypeInfo() const {
        return "VertexPose";
    }
};


}
#endif