#include <iostream>
#include <fstream>

#include <benchmark/benchmark.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

static void BM_Pinocchio_RNEA(benchmark::State& state)
{
  se3::Model model;

  // Load an urdf file provided by the user
  se3::urdf::buildModel("models/simple_humanoid.urdf", se3::JointModelFreeFlyer(), model);
  std::cout << "Pinocchio Benchmark" << std::endl;
  std::cout << "  model: " << "models/simple_humanoid.urdf" << std::endl;
  std::cout << "  nq: " << model.nq << std::endl;
  std::cout << "  nv: " << model.nv << std::endl;

  se3::Data data(model);

  for (auto _: state)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);
    Eigen::VectorXd qdot = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd qddot = Eigen::VectorXd::Random(model.nv);

    se3::aba(model, data, q, qdot, tau);
    //std::cout << "qddot after ABA: " << data.ddq.transpose() << std::endl;
  }
}

BENCHMARK(BM_Pinocchio_RNEA);

BENCHMARK_MAIN();
