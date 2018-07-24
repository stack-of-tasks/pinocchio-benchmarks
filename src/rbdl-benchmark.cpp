#include <iostream>
#include <fstream>

#include <benchmark/benchmark.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

static void BM_RBDL_RNEA(benchmark::State& state)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();

  // Load an urdf file provided by the user
  RigidBodyDynamics::Addons::URDFReadFromFile("models/simple_humanoid.urdf", model, true);
  std::cout << "RBDL Benchmark" << std::endl;
  std::cout << "  model: " << "models/simple_humanoid.urdf" << std::endl;
  std::cout << "  nq: " << model->q_size << std::endl;
  std::cout << "  nv: " << model->qdot_size << std::endl;

  RigidBodyDynamics::Math::VectorNd q =
    RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
  RigidBodyDynamics::Math::VectorNd qdot =
    RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd tau =
    RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd qddot =
    RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);

  for (auto _: state)
  {
    q = RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
    qdot = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    tau = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    qddot = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);

    RigidBodyDynamics::ForwardDynamics(*model, q, qdot, tau, qddot);
    //std::cout << "qddot after ABA: " << qddot.transpose() << std::endl;
  }
}

BENCHMARK(BM_RBDL_RNEA);

BENCHMARK_MAIN();
