#include <iostream>
#include <fstream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <pinocchio/tools/timer.hpp>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

#include "models.h"

#define NBT 100 * 1000

void benchmark_pinocchio_rnea(std::string model_file)
{
  StackTicToc timer(StackTicToc::NS);

  se3::Model model;
  se3::urdf::buildModel(pinocchio_benchmarks::path + model_file,
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

  timer.tic();
  for(size_t i=0; i<NBT; i++) se3::rnea(model, data, qs[i], qdots[i], taus[i]);
  std::cout << "Pinocchio RNEA " << model_file << " \t ";
  timer.toc(std::cout, NBT);
}

void benchmark_pinocchio_aba(std::string model_file)
{
  StackTicToc timer(StackTicToc::NS);

  se3::Model model;
  se3::urdf::buildModel(pinocchio_benchmarks::path + model_file,
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

  timer.tic();
  for(size_t i=0; i<NBT; i++) se3::aba(model, data, qs[i], qdots[i], taus[i]);
  std::cout << "Pinocchio ABA  " << model_file << " \t ";
  timer.toc(std::cout, NBT);
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_pinocchio_rnea(model);
  for (auto& model : pinocchio_benchmarks::models)
    benchmark_pinocchio_aba(model);

  return 0;
}
