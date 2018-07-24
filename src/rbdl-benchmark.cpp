#include <iostream>
#include <fstream>

#include <benchmark/benchmark.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

static void BM_RBDL_RNEA(benchmark::State& state)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile("models/simple_humanoid.urdf", model, true);

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
}

static void BM_RBDL_ABA(benchmark::State& state)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile("models/simple_humanoid.urdf", model, true);

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
}


BENCHMARK(BM_RBDL_RNEA);
BENCHMARK(BM_RBDL_ABA);

BENCHMARK_MAIN();
