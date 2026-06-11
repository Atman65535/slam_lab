# pragma once
#include <iostream>
#include "structure.hpp"

// test passed
// ABORTED
M3D hat(const V3D vector);
M3D asymmetric(const V3D vector);
V3D vee_vector(const M3D m);

M3D Exp(const V3D lie_algebra);
V3D Log(const M3D R);

StatusManifold boxplus(const StatusManifold& s, const StatusTangent& t);

// BCH
M3D SO3_left_jacobian(const V3D main_part);
M3D SO3_left_jacobian_inv(const V3D main_part);
M3D SO3_right_jacobian(const V3D main_part);
M3D SO3_right_jacobian_inv(const V3D main_part);
