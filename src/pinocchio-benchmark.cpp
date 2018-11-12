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
  pinocchio::Model model;
  pinocchio::urdf::buildModel(pinocchio_benchmarks::path +
      pinocchio_benchmarks::models[(unsigned long)state.range(0)] + ".urdf",
      pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Zero(model.nv);

  for (auto _: state)
  {
    state.PauseTiming();
    q = Eigen::VectorXd::Random(model.nq);
    q.segment(0, 4).normalize();
    qdot = Eigen::VectorXd::Random(model.nv);
    qddot = Eigen::VectorXd::Random(model.nv);
    state.ResumeTiming();

    pinocchio::rnea(model, data, q, qdot, tau);
  }
}

static void BM_Pinocchio_ABA(benchmark::State& state)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModel(pinocchio_benchmarks::path +
      pinocchio_benchmarks::models[(unsigned long)state.range(0)] + ".urdf",
      pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd qddot = Eigen::VectorXd::Zero(model.nv);

  for (auto _: state)
  {
    state.PauseTiming();
    q = Eigen::VectorXd::Random(model.nq);
    q.segment(0, 4).normalize();
    qdot = Eigen::VectorXd::Random(model.nv);
    tau = Eigen::VectorXd::Random(model.nv);
    state.ResumeTiming();

    pinocchio::aba(model, data, q, qdot, tau);
  }
}

BENCHMARK(BM_Pinocchio_RNEA)->Arg(0)->Arg(1)->Arg(2)->Arg(3)->Arg(4)->Arg(5)->Arg(6);
BENCHMARK(BM_Pinocchio_ABA)->Arg(0)->Arg(1)->Arg(2)->Arg(3)->Arg(4)->Arg(5)->Arg(6);

BENCHMARK_MAIN();
