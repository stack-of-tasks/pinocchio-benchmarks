#include <chrono>
#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::VectorNd)

#include "models.h"

void benchmark_rbdl_rnea(std::string model, std::string log_filename)
{
  RigidBodyDynamics::Model* robot = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model + ".urdf").c_str(), robot,
      pinocchio_benchmarks::free_flyer(model));

  std::vector<RigidBodyDynamics::Math::VectorNd> qs(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qdots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qddots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->q_size);
    if (robot->q_size > robot->qdot_size) qs[i].segment(0, 4).normalize();
    qdots[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->qdot_size);
    qddots[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->qdot_size);
    taus[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->qdot_size);
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for(size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    RigidBodyDynamics::InverseDynamics(*robot, qs[i], qdots[i], qddots[i], taus[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();

  delete robot;
}

void benchmark_rbdl_crba(std::string model, std::string log_filename)
{
  RigidBodyDynamics::Model* robot = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model + ".urdf").c_str(), robot,
      pinocchio_benchmarks::free_flyer(model));

  RigidBodyDynamics::Math::MatrixNd h =
    RigidBodyDynamics::Math::MatrixNd::Constant(
        robot->dof_count, robot->dof_count, 0);

  std::vector<RigidBodyDynamics::Math::VectorNd> qs(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->q_size);
    if (robot->q_size > robot->qdot_size) qs[i].segment(0, 4).normalize();
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for(size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*robot, qs[i], h);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();

  delete robot;
}

void benchmark_rbdl_aba(std::string model, std::string log_filename)
{
  RigidBodyDynamics::Model* robot = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model + ".urdf").c_str(), robot,
      pinocchio_benchmarks::free_flyer(model));

  std::vector<RigidBodyDynamics::Math::VectorNd> qs(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qdots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> qddots(NBT);
  std::vector<RigidBodyDynamics::Math::VectorNd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->q_size);
    if (robot->q_size > robot->qdot_size) qs[i].segment(0, 4).normalize();
    qdots[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->qdot_size);
    qddots[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->qdot_size);
    taus[i] = RigidBodyDynamics::Math::VectorNd::Random(robot->qdot_size);
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for(size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    RigidBodyDynamics::ForwardDynamics(*robot, qs[i], qdots[i], taus[i], qddots[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();

  delete robot;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    benchmark_rbdl_rnea(model, pinocchio_benchmarks::get_log_filename(
          "RBDL", "ID", model));
    benchmark_rbdl_crba(model, pinocchio_benchmarks::get_log_filename(
          "RBDL", "FD", model));
    benchmark_rbdl_aba(model, pinocchio_benchmarks::get_log_filename(
          "RBDL", "HD", model));
  }

  return 0;
}
