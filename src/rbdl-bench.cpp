#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

int main(int argc, char** argv)
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

  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();

  // Load an urdf file provided by the user
  RigidBodyDynamics::Addons::URDFReadFromFile(argv[1], model, true);
  std::cout << "RBDL Benchmark" << std::endl;
  std::cout << "  model: " << argv[1] << std::endl;
  std::cout << "  nq: " << model->q_size << std::endl;
  std::cout << "  nv: " << model->qdot_size << std::endl;

  RigidBodyDynamics::Math::VectorNd q =
    RigidBodyDynamics::Math::VectorNd::Random(model->q_size);
  RigidBodyDynamics::Math::VectorNd qdot =
    RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd tau =
    RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd qddot =
    RigidBodyDynamics::Math::VectorNd::Random(model->qdot_size);

  RigidBodyDynamics::ForwardDynamics(*model, q, qdot, tau, qddot);
  std::cout << "qddot after ABA: " << qddot.transpose() << std::endl;

  RigidBodyDynamics::InverseDynamics(*model, q, qdot, qddot, tau);
  std::cout << "tau aftern RNEA: " << tau.transpose() << std::endl;

  delete model;
  return 0;
}
