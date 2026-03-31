import casadi as ca
import math

# symbolic inputs
current_vel = ca.MX.sym("current_vel", 3)
desired_vel = ca.MX.sym("desired_vel", 3)

lerp_xy = ca.MX.sym("lerp_xy")
lerp_rot = ca.MX.sym("lerp_rot")

mod_positions = [
    [0.4, 0.4],
    [-0.4, 0.4],
    [-0.4, -0.4],
    [0.4, -0.4],
]

max_mod_vel = 4

# derived chassis state
desired_chassis = ca.vertcat(
    current_vel[0] + lerp_xy * (desired_vel[0] - current_vel[0]),
    current_vel[1] + lerp_xy * (desired_vel[1] - current_vel[1]),
    current_vel[2] + lerp_rot * (desired_vel[2] - current_vel[2]),
)

# module velocities
constraints = []
for i in range(4):
    vx = desired_chassis[0] + desired_chassis[2] * -mod_positions[i][1]
    vy = desired_chassis[1] + desired_chassis[2] * mod_positions[i][0]
    constraints.append(vx**2 + vy**2 - max_mod_vel**2)

g = ca.vertcat(*constraints)

# cost
trans_delta_mag = ca.norm_2(desired_vel[0:2] - current_vel[0:2])
rot_delta_mag = ca.fabs(desired_vel[2] - current_vel[2])

cost = -(lerp_xy * trans_delta_mag + 5 * lerp_rot * rot_delta_mag)

# decision vector
x = ca.vertcat(lerp_xy, lerp_rot)

# build function
f = ca.Function("f", [x, current_vel, desired_vel], [cost, g])

nlp = {
    "x": x,
    "f": cost,
    "g": g,
    "p": ca.vertcat(current_vel, desired_vel)
}

solver = ca.nlpsol("solver", "sqpmethod", nlp,     {
        "qpsol": "qrqp",   # <-- this is the magic
        "print_header": True,
        "print_iteration": True,
        "print_time": True,
    })
solver.generate("solver.c")