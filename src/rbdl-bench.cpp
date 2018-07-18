#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;
using namespace RigidBodyDynamics::Math;

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "You have to specify a path to an urdf file" << std::endl;
    return 1;
  }

  std::ifstream path(argv[1]);
  if (!path)
  {
    std::cerr << "This path is not a valid file: " << argv[1] << std::endl;
    return 2;
  }

  Model* model = new Model();

  // Load an urdf file provided by the user, with a floating base
  URDFReadFromFile(argv[1], model, true);

  VectorNd Q = VectorNd::Zero (model->dof_count);
  VectorNd QDot = VectorNd::Zero (model->dof_count);
  VectorNd Tau = VectorNd::Zero (model->dof_count);
  VectorNd QDDot = VectorNd::Zero (model->dof_count);

  ForwardDynamics (*model, Q, QDot, Tau, QDDot);
  std::cout << "QDDot after ABA: " << QDDot.transpose() << std::endl;

  InverseDynamics (*model, Q, QDot, QDDot, Tau);
  std::cout << "Tau aftern RNEA: " << Tau.transpose() << std::endl;

  delete model;
  return 0;
}
