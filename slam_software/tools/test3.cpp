#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

int main() {
    Eigen::MatrixXd m ( {{0, 1, 0}, {0.2, 2, 0.03}, {-0.12, -2, -0.3}, {-0.1, 3, -0.2}, {0, 1, -0.1}});
    m = m.transpose() * m;
    // stored in increasing order
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(m);  
    std::cout << es.eigenvalues() << std::endl;
    std::cout << es.eigenvectors() << std::endl;
    std::cout << es.eigenvectors()(2, 2) << std::endl;
    std::cout << es.eigenvectors().col(2) << std::endl;
    std::cout << es.eigenvalues()(2) << std::endl;
    Eigen::Matrix<double, Eigen::Dynamic, 18> H;
    H.resize(3, 18);
    H.setZero();
    std::cout << H << std::endl;
}
