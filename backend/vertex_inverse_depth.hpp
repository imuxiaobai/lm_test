#ifndef VERTEX_INVERSE_DEPTH_H
#define VERTEX_INVERSE_DEPTH_H
#include "vertex.hpp"
using namespace Eigen;

namespace lm_test{
/* 逆深度 */
class VertexInverseDepth : public Vertex {
public:
    VertexInverseDepth() : Vertex(1) {}

    std::string TypeInfo() const {
        return "VertexInverseDepth";
    }
};


}
#endif