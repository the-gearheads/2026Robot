import casadi as ca
import math
#wpilib coordinate space, +X is forward, +Y is left, CCW+

mod_positions = [
    [0.4, 0.4],
    [-0.4, 0.4],
    [-0.4, -0.4],
    [0.4, -0.4],
]


max_mod_vel = 4

p = ca.Opti()

current_vel: ca.MX = p.parameter(3)
desired_vel: ca.MX = p.parameter(3)

p.set_value(current_vel, [4, 0, 0])
p.set_value(desired_vel, [10, 0, math.radians(9000)])

trans_delta_mag = ca.hypot(desired_vel[0] - current_vel[0], desired_vel[1] - current_vel[1])
rot_delta_mag = ca.fabs(desired_vel[2] - current_vel[2])


desired_chassis_state: list[ca.MX] = []
desired_mod_states: list[list[ca.MX]] = []

lerp_xy = p.variable()
lerp_rot = p.variable()

p.set_initial(lerp_xy, 1)
p.set_initial(lerp_rot, 1)

desired_chassis_state.append(current_vel[0] + lerp_xy  * (desired_vel[0] - current_vel[0]))
desired_chassis_state.append(current_vel[1] + lerp_xy  * (desired_vel[1] - current_vel[1]))
desired_chassis_state.append(current_vel[2] + lerp_rot * (desired_vel[2] - current_vel[2]))

p.subject_to(lerp_xy >= 0)
p.subject_to(lerp_xy <= 1)
p.subject_to(lerp_rot >= 0)
p.subject_to(lerp_rot <= 1)

# p.maximize(lerp_xy**2 + (5 * lerp_rot**2)) # weigh against reducing rotational speed more
# p.minimize((lerp_xy**2 + (5 * lerp_rot**2)) * -1)
# p.minimize( trans_delta_mag * (1 - lerp_xy)**2 + (5 * rot_delta_mag) * (1 - lerp_rot)**2 )
# gotta be quadratic or else qrqp gets very mad at you and probably wants your firstborn child
p.minimize(
    1 * (trans_delta_mag * (1 - lerp_xy))**2 + 
    5 * (rot_delta_mag * (1 - lerp_rot))**2
)

for i in range(4):
    # basically, mod_v = chassis_v + w * perp(mod_pos)
    mod_vx = desired_chassis_state[0] + desired_chassis_state[2] * -mod_positions[i][1]
    mod_vy = desired_chassis_state[1] + desired_chassis_state[2] *  mod_positions[i][0]
    desired_mod_states.append([mod_vx, mod_vy])

    # max drive velocity constraint
    p.subject_to(mod_vx**2 + mod_vy**2 <= max_mod_vel**2)


# p.solver("ipopt") # this one is cool and good but like... ew dependencies and slow
p.solver("sqpmethod", {"qpsol": "qrqp"})

sol = p.solve()

# not used in the codegen lmao
def mod_states_to_polar_components(mod_states):
    out = []
    for i in range(4):
        vx = sol.value(mod_states[i][0])
        vy = sol.value(mod_states[i][1])
        r = math.hypot(vx, vy)
        theta = math.degrees(math.atan2(vy, vx))
        out.append([r, theta])
    return out

print("final output")
print(f"original desired state: {desired_vel}")
print(f"corrected chassis state: {sol.value(desired_chassis_state[0])}, {sol.value(desired_chassis_state[1])}, {math.degrees(sol.value(desired_chassis_state[2]))} deg/s")
print(f"desired mod states: {mod_states_to_polar_components(desired_mod_states)}")
print(f"lerp constants: {sol.value(lerp_xy)}, {sol.value(lerp_rot)}")

# now for fun
f = p.to_function(
    "max_swerve_lerp",
    [current_vel, desired_vel],   # inputs
    [
        lerp_xy,
        lerp_rot
    ]                              # outputs
)

# solver = ca.nlpsol("solver", "sqpmethod", nlp,     {
#         "qpsol": "qrqp",   # <-- this is the magic
#         "print_header": True,
#         "print_iteration": True,
#         "print_time": True,
#     })
# solver.generate("solver.c")

import os
import shutil

# ARGHHHHHH
outpath = f"{os.path.dirname(os.path.realpath(__file__))}/src/main/driver"
f.generate("solver.c", {
    "with_header": True,
})

shutil.move("solver.c", f"{outpath}/cpp/solver.c")
shutil.move("solver.h", f"{outpath}/include/solver.h")