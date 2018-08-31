#include <chrono>
#include <iostream>
#include <fstream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

#include "models.h"

void benchmark_pinocchio_rnea(std::string model, std::string log_filename)
{
  se3::Model robot;
  if (pinocchio_benchmarks::free_flyer(model) && model != "atlas")
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        se3::JointModelFreeFlyer(), robot);
  }
  else
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        robot);
  }
  se3::Data data(robot);

  std::vector<Eigen::VectorXd> qs(NBT);
  std::vector<Eigen::VectorXd> qdots(NBT);
  std::vector<Eigen::VectorXd> qddots(NBT);

  for (size_t i=0; i<NBT; i++)
  {
    qs[i] = Eigen::VectorXd::Random(robot.nq);
    if (robot.nq > robot.nv) qs[i].segment(0, 4).normalize();
    qdots[i] = Eigen::VectorXd::Random(robot.nv);
    qddots[i] = Eigen::VectorXd::Random(robot.nv);
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for (size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    se3::rnea(robot, data, qs[i], qdots[i], qddots[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

void benchmark_pinocchio_aba(std::string model, std::string log_filename)
{
  se3::Model robot;
  if (pinocchio_benchmarks::free_flyer(model) && model != "atlas")
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        se3::JointModelFreeFlyer(), robot);
  }
  else
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        robot);
  }
  se3::Data data(robot);

  std::vector<Eigen::VectorXd> qs(NBT);
  std::vector<Eigen::VectorXd> qdots(NBT);
  std::vector<Eigen::VectorXd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = Eigen::VectorXd::Random(robot.nq);
    if (robot.nq > robot.nv) qs[i].segment(0, 4).normalize();
    qdots[i] = Eigen::VectorXd::Random(robot.nv);
    taus[i] = Eigen::VectorXd::Random(robot.nv);
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for(size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    se3::aba(robot, data, qs[i], qdots[i], taus[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

void benchmark_pinocchio_crba(std::string model, std::string log_filename)
{
  se3::Model robot;
  if (pinocchio_benchmarks::free_flyer(model) && model != "atlas")
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        se3::JointModelFreeFlyer(), robot);
  }
  else
  {
    se3::urdf::buildModel(pinocchio_benchmarks::path + model + ".urdf",
        robot);
  }
  se3::Data data(robot);

  std::vector<Eigen::VectorXd> qs(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = Eigen::VectorXd::Random(robot.nq);
    if (robot.nq > robot.nv) qs[i].segment(0, 4).normalize();
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for(size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    se3::crba(robot, data, qs[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    benchmark_pinocchio_rnea(model, pinocchio_benchmarks::get_log_filename(
          "Pinocchio", "ID", model));
    benchmark_pinocchio_aba(model, pinocchio_benchmarks::get_log_filename(
          "Pinocchio", "HD", model));
    benchmark_pinocchio_crba(model, pinocchio_benchmarks::get_log_filename(
          "Pinocchio", "FD", model));
  }

  return 0;
}
