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
  se3::urdf::buildModel("models/simple_humanoid.urdf", se3::JointModelFreeFlyer(), model);
  se3::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Zero(model.nv);

  for (auto _: state)
  {
    state.PauseTiming();
    q = Eigen::VectorXd::Random(model.nq);
    qdot = Eigen::VectorXd::Random(model.nv);
    qddot = Eigen::VectorXd::Random(model.nv);
    state.ResumeTiming();

    se3::rnea(model, data, q, qdot, tau);
  }
}


static void BM_Pinocchio_ABA(benchmark::State& state)
{
  se3::Model model;
  se3::urdf::buildModel("models/simple_humanoid.urdf", se3::JointModelFreeFlyer(), model);
  se3::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Zero(model.nv);

  for (auto _: state)
  {
    state.PauseTiming();
    q = Eigen::VectorXd::Random(model.nq);
    qdot = Eigen::VectorXd::Random(model.nv);
    tau = Eigen::VectorXd::Random(model.nv);
    state.ResumeTiming();

    se3::aba(model, data, q, qdot, tau);
  }
}

BENCHMARK(BM_Pinocchio_RNEA);
BENCHMARK(BM_Pinocchio_ABA);

BENCHMARK_MAIN();
