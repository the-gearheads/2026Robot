#include "driver.h"
#include "solver.h"
#include <fmt/printf.h>
std::vector<double> solveSwerve(std::vector<double> current_vel, std::vector<double> desired_vel) {
  const casadi_real* arg[max_swerve_lerp_SZ_ARG];
  casadi_real* res[max_swerve_lerp_SZ_RES];
  casadi_int iw[max_swerve_lerp_SZ_IW];
  casadi_real w[max_swerve_lerp_SZ_W];

  //ARGHHHH
  double current_vel_c[] = {current_vel[0], current_vel[1], current_vel[2]};
  double desired_vel_c[] = {desired_vel[0], desired_vel[1], desired_vel[2]};
  double out_lerp_xy = 0.0;
  double out_lerp_rot = 0.0;

  // should match the order in the python or else evil will occur
  arg[0] = current_vel_c;
  arg[1] = desired_vel_c;

  res[0] = &out_lerp_xy;
  res[1] = &out_lerp_rot;

  // 4. Run the solver! 
  // The '0' at the end tells CasADi to use its default, single-threaded memory <---- says gemini
  max_swerve_lerp(arg, res, iw, w, 0);

  fmt::println("cpp side here!");
  fmt::println("out_lerp_xy: {}", out_lerp_xy);
  fmt::println("out_lerp_rot: {}", out_lerp_rot);
  fmt::println("that's all!");
  
  return {out_lerp_xy, out_lerp_rot};
}