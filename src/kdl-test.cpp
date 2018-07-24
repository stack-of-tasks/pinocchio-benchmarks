#include <iostream>
#include <fstream>

#include <kdl_parser/kdl_parser.hpp>

#include "models.h"

void kdl_test(std::string model_file)
{
  KDL::Tree tree;
  kdl_parser::treeFromFile(pinocchio_benchmarks::path + model_file, tree);

  std::cout << "KDL Test" << std::endl;
  std::cout << "  model: " << model_file << std::endl;
  std::cout << "  nq: " << tree.getNrOfJoints() << std::endl;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    kdl_test(model);
  }
  return 0;
}
