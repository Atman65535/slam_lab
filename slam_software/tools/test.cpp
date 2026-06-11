#include <Eigen/Core>
#include "so3_math.hpp"
#include "structure.hpp"
#include <Eigen/Eigenvalues>

int main() {
    M3D R = Exp(V3D (1, 2, 3));
    Eigen::EigenSolver<M3D> solver (R);
    std::cout << solver.eigenvectors() << std::endl;
}

