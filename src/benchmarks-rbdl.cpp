#include <chrono>
#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::VectorNd)

#include "models.h"

void benchmark_rbdl_rnea(std::string model_file, std::string filename)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model_file + ".urdf").c_str(), model, true);

  std::vector<RigidBodyDynamics::Math::VectorNd> qs(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qdots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qddots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
    qs[i].segment(0, 4).normalize();
    qdots[i] = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    qddots[i] = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    taus[i] = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
  }

  std::ofstream file;
  file.open(filename);
  for(size_t i=0; i<NBT; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    RigidBodyDynamics::InverseDynamics(*model, qs[i], qdots[i], qddots[i], taus[i]);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();

  delete model;
}

void benchmark_rbdl_aba(std::string model_file, std::string filename)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model_file + ".urdf").c_str(), model, true);

  std::vector<RigidBodyDynamics::Math::VectorNd> qs(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qdots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qddots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
    qs[i].segment(0, 4).normalize();
    qdots[i] = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    qddots[i] = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
    taus[i] = RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
  }

  std::ofstream file;
  file.open(filename);
  for(size_t i=0; i<NBT; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    RigidBodyDynamics::ForwardDynamics(*model, qs[i], qdots[i], taus[i], qddots[i]);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();

  delete model;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_rbdl_rnea(model, pinocchio_benchmarks::get_filename(
          "RBDL", "RNEA", model));
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_rbdl_aba(model, pinocchio_benchmarks::get_filename(
          "RBDL", "ABA", model));

  return 0;
}
