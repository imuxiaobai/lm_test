#ifndef VERTEX_MOTION_H
#define VERTEX_MOTION_H
#include "vertex.hpp"
using namespace Eigen;

namespace lm_test{
/*v, ba, bg */
class VertexMotion : public Vertex {
public:
    VertexMotion() : Vertex(9) {}

    std::string TypeInfo() const {
        return "VertexMotion";
    }
};


}
#endif