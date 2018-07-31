#include <iostream>
#include <fstream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include "models.h"

void pinocchio_test(std::string model_file)
{
  se3::Model model;
  se3::urdf::buildModel(pinocchio_benchmarks::path + model_file,
      se3::JointModelFreeFlyer(), model);

  std::cout << "Pinocchio Test" << std::endl;
  std::cout << "  model: " << model_file << std::endl;
  std::cout << "  nq: " << model.nq << std::endl;
  std::cout << "  nv: " << model.nv << std::endl;

  se3::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Zero(model.nv);
  q(0) = 1;
  q.segment(0, 4).normalize();

  se3::aba(model, data, q, qdot, tau);
  std::cout << "qddot after ABA: " << data.ddq.transpose() << std::endl;

  se3::rnea(model, data, q, qdot, qddot);
  std::cout << "tau aftern RNEA: " << data.tau.transpose() << std::endl;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    pinocchio_test(model);
  }
  return 0;
}
