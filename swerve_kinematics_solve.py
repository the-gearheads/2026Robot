from sleipnir.optimization import Problem
from sleipnir.autodiff import Variable, VariableMatrix, hypot
import math
#wpilib coordinate space, +X is forward, +Y is left, CCW+

mod_positions = [
    [0.4, 0.4],
    [-0.4, 0.4],
    [-0.4, -0.4],
    [0.4, -0.4],
]


current_vel = [1, 0, 0]
desired_vel = [10, 0, math.radians(9000)]

trans_delta_mag = math.hypot(desired_vel[0] - current_vel[0], desired_vel[1] - current_vel[1])
rot_delta_mag = abs(desired_vel[2] - current_vel[2])


max_mod_vel = 4

p = Problem()

# current_chassis_state: Variable = p.decision_variable(3) # vx, vy, w
desired_chassis_state: Variable = p.decision_variable(3) # vx, vy, w
desired_mod_states: VariableMatrix = p.decision_variable(4, 2) # vx, vy

lerp_xy = p.decision_variable()
lerp_rot = p.decision_variable()

p.subject_to(desired_chassis_state[0] == current_vel[0] + lerp_xy * (desired_vel[0] - current_vel[0]))
p.subject_to(desired_chassis_state[1] == current_vel[1] + lerp_xy * (desired_vel[1] - current_vel[1]))
p.subject_to(desired_chassis_state[2] == current_vel[2] + lerp_rot * (desired_vel[2] - current_vel[2]))

p.subject_to(lerp_xy >= 0)
p.subject_to(lerp_xy <= 1)
p.subject_to(lerp_rot >= 0)
p.subject_to(lerp_rot <= 1)

# p.maximize(lerp_xy**2 + (5 * lerp_rot**2)) # weigh against reducing rotational speed more
p.maximize(lerp_xy * trans_delta_mag + (5 * lerp_rot * rot_delta_mag))
for i in range(4):
    # basically, mod_v = chassis_v + w * perp(mod_pos)
    p.subject_to(desired_mod_states[i, 0] == desired_chassis_state[0] + desired_chassis_state[2] * -mod_positions[i][1])
    p.subject_to(desired_mod_states[i, 1] == desired_chassis_state[1] + desired_chassis_state[2] * mod_positions[i][0])
    # max drive velocity constraint
    p.subject_to(desired_mod_states[i, 0]**2 + desired_mod_states[i, 1]**2 <= max_mod_vel**2)

p.solve(diagnostics=True)

def mod_states_to_polar_components(mod_states):
    out = []
    for i in range(4):
        vx = mod_states[i, 0].value()
        vy = mod_states[i, 1].value()
        r = math.hypot(vx, vy)
        theta = math.degrees(math.atan2(vy, vx))
        out.append([r, theta])
    return out

print("final output")
print(f"original desired state: {desired_vel}")
print(f"corrected chassis state: {desired_chassis_state.value()}")
print(f"desired mod states: {mod_states_to_polar_components(desired_mod_states)}")
print(f"lerp constants: {lerp_xy.value()}, {lerp_rot.value()}")