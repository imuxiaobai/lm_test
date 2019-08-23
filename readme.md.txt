# 说明

2019-07-25
实现了针对一个节点的优化

还差的
1.先验信息
2.ordering_id现在还是从0开始，并没有写这块的逻辑
3.只写了添加节点和边，还没有写删除点和边
4.求delta_x还只用了H-1*b,应写成schur
5.vertex 的update未写广义的加法

6.local_dimension  代表可自由变化的维度，如旋转矩阵的位姿 num_dimension是12,但是local_dimension是6