#include <iostream>
#include <fstream>

#include <benchmark/benchmark.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "models.h"

static void BM_RBDL_RNEA(benchmark::State& state)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      pinocchio_benchmarks::models[state.range(0)]).c_str(), model, true);

  RigidBodyDynamics::Math::VectorNd q =
    RigidBodyDynamics::Math::VectorNd::Zero(model->q_size);
  RigidBodyDynamics::Math::VectorNd qdot =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd tau =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd qddot =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);

  for (auto _: state)
  {
    state.PauseTiming();
    q = RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
    qdot = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    qddot = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    state.ResumeTiming();

    RigidBodyDynamics::InverseDynamics(*model, q, qdot, tau, qddot);
  }

  delete model;
}

static void BM_RBDL_ABA(benchmark::State& state)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      pinocchio_benchmarks::models[state.range(0)]).c_str(), model, true);

  RigidBodyDynamics::Math::VectorNd q =
    RigidBodyDynamics::Math::VectorNd::Zero(model->q_size);
  RigidBodyDynamics::Math::VectorNd qdot =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd tau =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd qddot =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);

  for (auto _: state)
  {
    state.PauseTiming();
    q = RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
    qdot = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    tau = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    state.ResumeTiming();

    RigidBodyDynamics::ForwardDynamics(*model, q, qdot, tau, qddot);
  }

  delete model;
}


BENCHMARK(BM_RBDL_RNEA)->Arg(0)->Arg(1)->Arg(2)->Arg(3);
BENCHMARK(BM_RBDL_ABA)->Arg(0)->Arg(1)->Arg(2)->Arg(3);

BENCHMARK_MAIN();
