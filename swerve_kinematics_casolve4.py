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
p.set_value(desired_vel, [10, 0, math.radians(900)])

#trans_delta_mag = ca.hypot(desired_vel[0] - current_vel[0], desired_vel[1] - current_vel[1])
#rot_delta_mag = ca.fabs(desired_vel[2] - current_vel[2])


targeted_vel: list[ca.MX] = [p.variable(), p.variable(), p.variable()]
targeted_mod_states: list[list[ca.MX]] = []



#desired_chassis_state.append(current_vel[0] + lerp_xy  * (desired_vel[0] - current_vel[0]))
#desired_chassis_state.append(current_vel[1] + lerp_xy  * (desired_vel[1] - current_vel[1]))
#desired_chassis_state.append(current_vel[2] + lerp_rot * (desired_vel[2] - current_vel[2]))


# gotta be quadratic or else qrqp gets very mad at you and probably wants your firstborn child
#p.minimize(
#    1 * (trans_delta_mag * (1 - lerp_xy))**2 + 
#    5 * (rot_delta_mag * (1 - lerp_rot))**2
#)


cpe_err = (targeted_vel[0] * desired_vel[1] - targeted_vel[1] * desired_vel[0])**2

p.minimize(
    (targeted_vel[0] - desired_vel[0])**2 +
    (targeted_vel[1] - desired_vel[1])**2 +
    10 * (targeted_vel[2] - targeted_vel[2])**2 + 
    # then also need to minimize angles between target and desired_vel
    0
)


for i in range(4):
    # basically, mod_v = chassis_v + w * perp(mod_pos)
    mod_vx = targeted_vel[0] + targeted_vel[2] * -mod_positions[i][1]
    mod_vy = targeted_vel[1] + targeted_vel[2] *  mod_positions[i][0]
    targeted_mod_states.append([mod_vx, mod_vy])

    # max drive velocity constraint
    p.subject_to(mod_vx**2 + mod_vy**2 <= max_mod_vel**2)


p.solver("ipopt") # this one is cool and good but like... ew dependencies and slow
#p.solver("sqpmethod", {"qpsol": "qrqp"})

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
print(f"corrected chassis state: {sol.value(targeted_vel[0])}, {sol.value(targeted_vel[1])}, {math.degrees(sol.value(targeted_vel[2]))} deg/s")
print(f"desired mod states: {mod_states_to_polar_components(targeted_mod_states)}")
#print(f"lerp constants: {sol.value(lerp_xy)}, {sol.value(lerp_rot)}")

# now for fun
f = p.to_function(
    "max_swerve_lerp",
    [current_vel, desired_vel],   # inputs
    [
        targeted_vel[0],
        targeted_vel[1],
        targeted_vel[2]
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
