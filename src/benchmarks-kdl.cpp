#include <chrono>
#include <iostream>
#include <fstream>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainidsolver_vereshchagin.hpp>

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(KDL::JntArray)

#include "models.h"

void benchmark_kdl_rnea(std::string model, std::string log_filename)
{
  KDL::Chain chain;
  KDL::Tree tree;
  kdl_parser::treeFromFile(pinocchio_benchmarks::path + model + ".urdf", tree);
  tree.getChain("world_link", "lwr_arm_7_link", chain);

  KDL::Wrenches f;
  KDL::Vector g_v(0, 0, -9.81);
  KDL::ChainIdSolver_RNE rnea_solver(chain, g_v);

  std::vector<KDL::JntArray> qs(NBT);
  std::vector<KDL::JntArray> qdots(NBT);
  std::vector<KDL::JntArray> qddots(NBT);
  std::vector<KDL::JntArray> taus(NBT);

  for (size_t i=0; i<NBT; i++)
  {
    qs[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
    qdots[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
    qddots[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
    taus[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for (size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    rnea_solver.CartToJnt(qs[i], qdots[i], qddots[i], f, taus[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

void benchmark_kdl_vc(std::string model, std::string log_filename)
{
  KDL::Chain chain;
  KDL::Tree tree;
  kdl_parser::treeFromFile(pinocchio_benchmarks::path + model + ".urdf", tree);
  tree.getChain("world_link", "lwr_arm_7_link", chain);

  KDL::Wrenches f;
  KDL::Vector g_v(0, 0, -9.81);
  KDL::Twist g_t(g_v, KDL::Vector::Zero());
  unsigned int nr_of_constraints = 4;
  KDL::Jacobian alpha(nr_of_constraints - 1);
  KDL::JntArray beta(nr_of_constraints - 1);
  KDL::ChainIdSolver_Vereshchagin vc_solver(chain, g_t, nr_of_constraints);

  std::vector<KDL::JntArray> qs(NBT);
  std::vector<KDL::JntArray> qdots(NBT);
  std::vector<KDL::JntArray> qddots(NBT);
  std::vector<KDL::JntArray> taus(NBT);

  for (size_t i=0; i<NBT; i++)
  {
    qs[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
    qdots[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
    qddots[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
    taus[i].data = Eigen::VectorXd::Random(chain.getNrOfJoints());
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::ofstream file(log_filename);
  for (size_t i=0; i<NBT; i++)
  {
    start = std::chrono::high_resolution_clock::now();
    vc_solver.CartToJnt(qs[i], qdots[i], qddots[i], alpha, beta, f, taus[i]);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = end - start;
    file << time.count() << std::endl;
  }
  file.close();
}

int main()
{
  std::string model = "lwr";  // KDL have dynamic solvers only for chains
  benchmark_kdl_rnea(model, pinocchio_benchmarks::get_log_filename(
        "KDL", "ID", model));
  benchmark_kdl_vc(model, pinocchio_benchmarks::get_log_filename(
        "KDL", "HD", model));

  return 0;
}
