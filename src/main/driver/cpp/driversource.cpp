#include "driverheader.h"
#include <sleipnir/optimization/problem.hpp>

void doThing() {
  // can we sleipnir?  
  slp::Problem problem;
  auto x = problem.decision_variable();
  auto y = problem.decision_variable();
  problem.minimize(x * x + y * y);
  problem.subject_to(x + y == 1);
  problem.solve({.diagnostics = true});
}