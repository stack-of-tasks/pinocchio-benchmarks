#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "models.h"

void rbdl_test(std::string model_file)
{
  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
      model_file + ".urdf").c_str(), model, true);

  std::cout << "RBDL Test" << std::endl;
  std::cout << "  model: " << model_file << std::endl;
  std::cout << "  nq: " << model->q_size << std::endl;
  std::cout << "  nv: " << model->qdot_size << std::endl;

  RigidBodyDynamics::Math::VectorNd q =
    RigidBodyDynamics::Math::VectorNd::Zero(model->q_size);
  RigidBodyDynamics::Math::VectorNd qdot =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd tau =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  RigidBodyDynamics::Math::VectorNd qddot =
    RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);
  q(0) = 1;
  q.segment(0, 4).normalize();

  RigidBodyDynamics::ForwardDynamics(*model, q, qdot, tau, qddot);
  std::cout << "qddot after ABA: " << qddot.transpose() << std::endl;

  qddot = RigidBodyDynamics::Math::VectorNd::Zero(model->qdot_size);

  RigidBodyDynamics::InverseDynamics(*model, q, qdot, qddot, tau);
  std::cout << "tau aftern RNEA: " << tau.transpose() << std::endl;

  delete model;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    rbdl_test(model);
  }
  return 0;
}
