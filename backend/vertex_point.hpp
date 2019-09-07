#ifndef VERTEX_POINT_H
#define VERTEX_POINT_H
#include "vertex.hpp"
using namespace Eigen;

namespace lm_test{
/*x, y, z 形式的point */
class VertexPoint : public Vertex {
public:
    VertexPoint() : Vertex(3) {}

    std::string TypeInfo() const {
        return "VertexPoint";
    }
};


}
#endif