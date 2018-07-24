#include <iostream>
#include <fstream>

#include <benchmark/benchmark.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include "models.h"

static void BM_Pinocchio_RNEA(benchmark::State& state)
{
  se3::Model model;
  se3::urdf::buildModel(pinocchio_benchmarks::path +
      pinocchio_benchmarks::models[state.range(0)],
      se3::JointModelFreeFlyer(), model);
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
  se3::urdf::buildModel(pinocchio_benchmarks::path +
      pinocchio_benchmarks::models[state.range(0)],
      se3::JointModelFreeFlyer(), model);
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

BENCHMARK(BM_Pinocchio_RNEA)->Arg(0)->Arg(1)->Arg(2)->Arg(3);
BENCHMARK(BM_Pinocchio_ABA)->Arg(0)->Arg(1)->Arg(2)->Arg(3);

BENCHMARK_MAIN();
