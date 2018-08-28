#include <chrono>
#include <iostream>
#include <fstream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

#include "models.h"

#define NBT 100 * 1000

void benchmark_pinocchio_rnea(std::string model_file, std::string filename)
{
  se3::Model model;
  se3::urdf::buildModel(pinocchio_benchmarks::path + model_file + ".urdf",
      se3::JointModelFreeFlyer(), model);
  se3::Data data(model);

  std::vector<Eigen::VectorXd> qs(NBT);
  std::vector<Eigen::VectorXd> qdots(NBT);
  std::vector<Eigen::VectorXd> qddots(NBT);
  std::vector<Eigen::VectorXd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = Eigen::VectorXd::Random(model.nq);
    qs[i].segment(0, 4).normalize();
    qdots[i] = Eigen::VectorXd::Random(model.nq);
    qddots[i] = Eigen::VectorXd::Random(model.nq);
    taus[i] = Eigen::VectorXd::Random(model.nq);
  }

  std::ofstream file;
  file.open(filename);
  for(size_t i=0; i<NBT; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    se3::rnea(model, data, qs[i], qdots[i], taus[i]);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

void benchmark_pinocchio_aba(std::string model_file, std::string filename)
{
  se3::Model model;
  se3::urdf::buildModel(pinocchio_benchmarks::path + model_file + ".urdf",
      se3::JointModelFreeFlyer(), model);
  se3::Data data(model);

  std::vector<Eigen::VectorXd> qs(NBT);
  std::vector<Eigen::VectorXd> qdots(NBT);
  std::vector<Eigen::VectorXd> qddots(NBT);
  std::vector<Eigen::VectorXd> taus(NBT);

  for(size_t i=0; i<NBT; i++)
  {
    qs[i] = Eigen::VectorXd::Random(model.nq);
    qs[i].segment(0, 4).normalize();
    qdots[i] = Eigen::VectorXd::Random(model.nq);
    qddots[i] = Eigen::VectorXd::Random(model.nq);
    taus[i] = Eigen::VectorXd::Random(model.nq);
  }

  std::ofstream file;
  file.open(filename);
  for(size_t i=0; i<NBT; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    se3::aba(model, data, qs[i], qdots[i], taus[i]);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_pinocchio_rnea(model, pinocchio_benchmarks::get_filename(
          "Pinocchio", "RNEA", model));
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_pinocchio_aba(model, pinocchio_benchmarks::get_filename(
          "Pinocchio", "ABA", model));

  return 0;
}
