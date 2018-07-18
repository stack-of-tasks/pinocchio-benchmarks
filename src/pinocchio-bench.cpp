#include <iostream>
#include <fstream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

int main(int argc, char** argv)
{

  if (argc != 2)
  {
    std::cerr << "You have to specify a path to an urdf file" << std::endl;
    return 1;
  }

  std::ifstream path(argv[1]);
  if (!path)
  {
    std::cerr << "This path is not a valid file: " << argv[1] << std::endl;
    return 2;
  }

  se3::Model model;

  // Load an urdf file provided by the user
  se3::urdf::buildModel(argv[1], se3::JointModelFreeFlyer(), model);
  std::cout << "Pinocchio Benchmark" << std::endl;
  std::cout << "  model: " << argv[1] << std::endl;
  std::cout << "  nq: " << model.nq << std::endl;
  std::cout << "  nv: " << model.nv << std::endl;

  se3::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Random(model.nv);

  se3::aba(model, data, q, qdot, tau);
  std::cout << "qddot after ABA: " << data.ddq.transpose() << std::endl;

  se3::rnea(model, data, q, qdot, qddot);
  std::cout << "tau aftern RNEA: " << data.tau.transpose() << std::endl;

  return 0;
}
