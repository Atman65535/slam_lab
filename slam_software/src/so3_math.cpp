#include "so3_math.hpp"
// test passed
// ABORTED
inline M3D hat(const V3D vector) {
    M3D m ({{0, -vector.z(), vector.y()},
            {vector.z(), 0, -vector.x()},
            {-vector.y(), vector.x(), 0}});
    return m;
}

inline M3D asymmetric(const V3D vector) {
    M3D m ({
           {0, -vector.z(), vector.y()},
           {vector.z(), 0, -vector.x()},
           {-vector.y(), vector.x(), 0}});
    return m;
}

inline V3D vee_vector(const M3D m) {
    if (m != -m.transpose()) {
        std::cout << "vee_vector: wrong m type!" << std::endl << m << std::endl; 
        throw std::format_error("wrong");
    }
    return V3D({m(2, 1), m(0, 2), m(1, 0)});
}

inline M3D Exp(const V3D lie_algebra) {
    double theta = lie_algebra.norm();
    M3D hat_u = hat(lie_algebra / theta);
    return (M3D::Identity() + hat_u * std::sin(theta) + hat_u * hat_u * (1 - std::cos(theta)));
}

// V3D Log(const M3D R) {
    
// }

StatusManifold boxplus(const StatusManifold& s, const StatusTangent& t) {
    StatusManifold m;
    m.R = s.R * Exp(t.dtheta);
    m.p = s.p + t.dp;
    m.v = s.v + t.dv;
    m.ba = s.ba + t.dba;
    m.bw = s.bw + t.dbw;
    m.g = s.g + t.dg;
    return m;
}

// refer to Vision SLAM 14 Lessons.
// usage: exp(\fai)exp(Jr \Delta \fai) = exp(\fai + \Delta fai)
inline M3D SO3_left_jacobian(const V3D main_part) {
    double theta = main_part.norm();
    V3D unified = main_part / theta;
    return (std::sin(theta) / theta) * M3D::Identity()
           + (1 - std::sin(theta) / theta) * unified * unified.transpose()
           + ((1 - std::cos(theta)) / theta) * hat(unified);
}

inline M3D SO3_left_jacobian_inv(const V3D main_part) {
    double theta = main_part.norm();
    V3D unified = main_part / theta;
    return (theta / 2) / std::tan(theta / 2) * M3D::Identity()
            + (1 - theta / 2 / tan(theta / 2)) * unified * unified.transpose()
            - theta / 2 * hat(unified);
}

inline M3D SO3_right_jacobian(const V3D main_part) {
    return SO3_left_jacobian(main_part).transpose();
}

inline M3D SO3_right_jacobian_inv(const V3D main_part) {
    return SO3_left_jacobian_inv(main_part).transpose();
}
