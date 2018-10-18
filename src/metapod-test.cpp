#include <iostream>
#include <fstream>

#include <metapod/algos/rnea.hh>
#include <pinocchio-benchmarks/metapod_simple/config.hh>
#include <pinocchio-benchmarks/metapod_simple/simple.hh>

#include "models.h"

int main()
{
  std::cout << "Metapod Test" << std::endl;
  std::cout << "  model: " << "simple" << std::endl;

  typedef simple<double> Robot;

  Robot robot;
  Robot::confVector q = Egine::Matrix<double, Robot.NBDOF, 1>::Zero();

  std::cout << "  nq: " << robot.NBDOF << std::endl;

  crba<Robot, true>::run(robot, q);
  std::cout << "state aftern CRBA: ";
  printState(robot, std::cout);

  rnea<Robot, true>::run(robot, q, qdot, qddot);
  std::cout << "tau aftern RNEA: ";
  printTorques<Robot>(robot, std::cout);

  return 0;
}
