#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "models.h"

void rbdl_test(std::string model)
{
  std::cout << "RBDL Test" << std::endl;
  std::cout << "  model: " << model << std::endl;

  RigidBodyDynamics::Model robot = RigidBodyDynamics::Model();
  RigidBodyDynamics::Addons::URDFReadFromFile((pinocchio_benchmarks::path +
        model + ".urdf").c_str(), &robot,
      pinocchio_benchmarks::free_flyer(model));

  std::cout << "  nq: " << robot.q_size << std::endl;
  std::cout << "  nv: " << robot.qdot_size << std::endl;

  RigidBodyDynamics::Math::VectorNd q =
    RigidBodyDynamics::Math::VectorNd::Zero(robot.q_size);
  RigidBodyDynamics::Math::VectorNd qdot =
    RigidBodyDynamics::Math::VectorNd::Zero(robot.qdot_size);
  RigidBodyDynamics::Math::VectorNd tau =
    RigidBodyDynamics::Math::VectorNd::Zero(robot.qdot_size);
  RigidBodyDynamics::Math::VectorNd qddot =
    RigidBodyDynamics::Math::VectorNd::Zero(robot.qdot_size);

  if (robot.q_size > robot.qdot_size)
  {
    q(0) = 1;
    q.segment(0, 4).normalize();
  }

  RigidBodyDynamics::Math::MatrixNd M =
    RigidBodyDynamics::Math::MatrixNd::Constant(
        robot.dof_count, robot.dof_count, 0);

  RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot, q, M);
  std::cout << "FD: M after CRBA: " << M(0,0) << std::endl;

  RigidBodyDynamics::ForwardDynamics(robot, q, qdot, tau, qddot);
  std::cout << "HD: qddot after ABA: " << qddot.transpose() << std::endl;

  qddot = RigidBodyDynamics::Math::VectorNd::Zero(robot.qdot_size);

  RigidBodyDynamics::InverseDynamics(robot, q, qdot, qddot, tau);
  std::cout << "ID: tau after RNEA: " << tau.transpose() << std::endl;
}

int main()
{
  for (auto& model : pinocchio_benchmarks::models)
  {
    rbdl_test(model);
  }
  return 0;
}
