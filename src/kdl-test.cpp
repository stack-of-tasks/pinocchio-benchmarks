#include <iostream>
#include <fstream>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainidsolver_vereshchagin.hpp>

#include "models.h"

void kdl_test(std::string model)
{
  std::cout << "KDL Test" << std::endl;
  std::cout << "  model: " << model << std::endl;

  KDL::Chain chain;
  KDL::Tree tree;
  kdl_parser::treeFromFile(pinocchio_benchmarks::path + model + ".urdf", tree);
  tree.getChain("world_link", "lwr_arm_7_link", chain);

  std::cout << "  nq: " << chain.getNrOfJoints() << std::endl;

  KDL::JntArray q(chain.getNrOfJoints());
  KDL::JntArray qdot(chain.getNrOfJoints());
  KDL::JntArray qddot(chain.getNrOfJoints());
  KDL::JntArray tau(chain.getNrOfJoints());
  KDL::Wrenches f;

  KDL::Vector g_v(0, 0, -9.81);
  KDL::Twist g_t(g_v, KDL::Vector::Zero());

  unsigned int nr_of_constraints = 4;
  KDL::Jacobian alpha(nr_of_constraints - 1);
  KDL::JntArray beta(nr_of_constraints - 1);

  KDL::ChainIdSolver_Vereshchagin vc_solver(chain, g_t, nr_of_constraints);
  vc_solver.CartToJnt(q, qdot, qddot, alpha, beta, f, tau);
  std::cout << "HD: qddot after VC: " << qddot.data.transpose() << std::endl;

  KDL::ChainIdSolver_RNE rnea_solver(chain, g_v);
  rnea_solver.CartToJnt(q, qdot, qddot, f, tau);
  std::cout << "ID: tau after RNEA: " << tau.data.transpose() << std::endl;
}

int main()
{
  kdl_test("lwr");
  return 0;
}
