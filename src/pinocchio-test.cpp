#include <iostream>
#include <fstream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include "models.h"

void pinocchio_test(std::string model)
{
  std::cout << "Pinocchio Test" << std::endl;
  std::cout << "  model: " << model << std::endl;

  se3::Model robot;
  if (pinocchio_benchmarks::free_flyer(model) && model != "atlas")
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        se3::JointModelFreeFlyer(), robot);
  }
  else
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        robot);
  }

  std::cout << "  nq: " << robot.nq << std::endl;
  std::cout << "  nv: " << robot.nv << std::endl;

  se3::Data data(robot);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(robot.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Zero(robot.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot.nv);

  if (robot.nq > robot.nv)
  {
    q(0) = 1;
    q.segment(0, 4).normalize();
  }

  se3::crba(robot, data, q);
  std::cout << "FD: M after CRBA: " << data.M(0,0) << std::endl;

  se3::aba(robot, data, q, qdot, tau);
  std::cout << "HD: qddot after ABA: " << data.ddq.transpose() << std::endl;

  se3::rnea(robot, data, q, qdot, qddot);
  std::cout << "ID: tau after RNEA: " << data.tau.transpose() << std::endl;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    pinocchio_test(model);
  }
  return 0;
}
