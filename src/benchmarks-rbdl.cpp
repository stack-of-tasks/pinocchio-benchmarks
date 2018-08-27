#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <pinocchio/tools/timer.hpp>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::VectorNd)

#include "models.h"

#define NBT 100 * 1000

void benchmark_rbdl_rnea(std::string model_file)
{
  StackTicToc timer(StackTicToc::NS);

  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model_file).c_str(), model, true);

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

  timer.tic();
  for(size_t i=0; i<NBT; i++)
    RigidBodyDynamics::InverseDynamics(*model, qs[i], qdots[i], qddots[i], taus[i]);
  std::cout << "RBDL      RNEA " << model_file << " \t ";
  timer.toc(std::cout, NBT);

  delete model;
}

void benchmark_rbdl_aba(std::string model_file)
{
  StackTicToc timer(StackTicToc::NS);

  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model_file).c_str(), model, true);

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

  timer.tic();
  for(size_t i=0; i<NBT; i++)
    RigidBodyDynamics::ForwardDynamics(*model, qs[i], qdots[i], taus[i], qddots[i]);
  std::cout << "RBDL      ABA  " << model_file << " \t ";
  timer.toc(std::cout, NBT);

  delete model;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_rbdl_rnea(model);
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_rbdl_aba(model);

  return 0;
}
