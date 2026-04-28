import numpy as np
import time
from scipy.optimize import minimize
import colorsys

# Constants and time constants
SAT_MAX = 0.95          # Saturation cap
F_VIS_ON = 0.95        # "Fully on" fraction of SAT_MAX
F_VIS_OFF = 0.05       # "Off" threshold

# Experimental time constants
tau_y_uv   = 58.3
tau_c_uv   = 25.8
tau_y_blue = 4.0
tau_c_red  = 55.0

# Reference color for dye blending
rgb_clear  = np.array([1.0, 1.0, 1.0])

rgb_yellow = np.array([151, 154, 67]) / 255.0   # #979A43
rgb_cyan   = np.array([39,  59,  100]) / 255.0  # #272F64

# Beer–Lambert multiplicative blend for green
rgb_green = rgb_yellow * rgb_cyan


def invert_color_to_saturations(hex_color):
    """
    Given a hex color, estimate (Sy, Sc) that best reproduce it
    under the forward physical dye-mixing model.
    """
    # Convert hex → RGB floats
    hex_color = hex_color.strip().lstrip("#")
    r = int(hex_color[0:2], 16) / 255.0
    g = int(hex_color[2:4], 16) / 255.0
    b = int(hex_color[4:6], 16) / 255.0
    target = np.array([r, g, b])

    def objective(x):
        Sy, Sc = x
        rgb = mix_color(Sy, Sc)
        return np.sum((rgb - target)**2)

    # Initial guess: use luminance to seed Sy or Sc bias
    x0 = np.array([0.3, 0.3])

    bounds = [(0.0, SAT_MAX), (0.0, SAT_MAX)]

    res = minimize(objective, x0, bounds=bounds, method="L-BFGS-B")

    Sy_est, Sc_est = res.x
    return float(Sy_est), float(Sc_est)

def mix_color(Sy, Sc):
    """
    Physically-inspired dye mixing model.
    Converts dye saturations into approximate RGB reflectance.
    """
    Sy_n = Sy / SAT_MAX
    Sc_n = Sc / SAT_MAX

    w_clear  = (1-Sy_n)*(1-Sc_n)
    w_yellow = Sy_n*(1-Sc_n)
    w_cyan   = Sc_n*(1-Sy_n)
    w_green  = Sy_n*Sc_n

    rgb = (w_clear*rgb_clear +
           w_yellow*rgb_yellow +
           w_cyan*rgb_cyan +
           w_green*rgb_green)

    return np.clip(rgb, 0, 1)


def rgb_to_hex(rgb):
    r = int(round(rgb[0]*255))
    g = int(round(rgb[1]*255))
    b = int(round(rgb[2]*255))
    return "#{:02X}{:02X}{:02X}".format(r, g, b)


def color_from_saturations(Sy, Sc):
    return rgb_to_hex(mix_color(Sy, Sc))

# UV activation time to reach target saturation
def uv_time_single(S_curr, S_target, tau):
    S_curr = np.clip(S_curr, 0.0, SAT_MAX)
    S_target = min(S_target, SAT_MAX)

    if S_target <= S_curr + 1e-9:
        return 0.0

    if S_target >= SAT_MAX - 1e-9:
        S_eff = F_VIS_ON * SAT_MAX
    else:
        S_eff = S_target

    denom = SAT_MAX - S_curr
    numer = SAT_MAX - S_eff
    if denom <= 0:
        return 0.0

    ratio = max(numer / denom, 1e-12)
    return -tau * np.log(ratio)


def uv_time_to_reach(Sy_curr, Sc_curr, Sy_tgt, Sc_tgt):
    return max(
        uv_time_single(Sy_curr, Sy_tgt, tau_y_uv),
        uv_time_single(Sc_curr, Sc_tgt, tau_c_uv)
    )


def erase_time(S_curr, S_target, tau):
    S_curr = np.clip(S_curr, 0, SAT_MAX)

    if S_target <= 1e-6:
        S_eff = F_VIS_OFF * SAT_MAX
    else:
        S_eff = min(S_target, S_curr)

    if S_eff >= S_curr - 1e-9:
        return 0.0

    return tau * np.log(S_curr / S_eff)

# UV target logic
def compute_uv_targets(Sy_tgt, Sc_tgt):
    UV_Sy = Sy_tgt if Sy_tgt > 0 else SAT_MAX
    UV_Sc = Sc_tgt if Sc_tgt > 0 else SAT_MAX
    return min(UV_Sy, SAT_MAX), min(UV_Sc, SAT_MAX)

# Sequential planner
def get_sequence_sequential(Sy_curr, Sc_curr, Sy_tgt, Sc_tgt):
    steps = []
    Sy, Sc = Sy_curr, Sc_curr

    need_y = Sy_tgt > Sy
    need_c = Sc_tgt > Sc

    if need_y or need_c:
        UV_Sy, UV_Sc = compute_uv_targets(Sy_tgt, Sc_tgt)
        t_uv = uv_time_to_reach(Sy, Sc, UV_Sy, UV_Sc)
        if t_uv > 0.01:
            steps.append(("UV", round(t_uv, 2)))

        Sy = SAT_MAX - (SAT_MAX - Sy)*np.exp(-t_uv / tau_y_uv)
        Sc = SAT_MAX - (SAT_MAX - Sc)*np.exp(-t_uv / tau_c_uv)

    if Sy > Sy_tgt + 1e-6:
        t_blue = erase_time(Sy, Sy_tgt, tau_y_blue)
        steps.append(("Blue", round(t_blue, 2)))
        Sy *= np.exp(-t_blue / tau_y_blue)

    if Sc > Sc_tgt + 1e-6:
        t_red = erase_time(Sc, Sc_tgt, tau_c_red)
        steps.append(("Red", round(t_red, 2)))
        Sc *= np.exp(-t_red / tau_c_red)

    if not steps:
        steps.append(("None", 0.0))

    return steps

# Kinetics
def combined_uv_blue(Sy, Sc, t):
    k_y = 1/tau_y_uv + 1/tau_y_blue
    A_y = SAT_MAX*(1/tau_y_uv) / k_y
    Sy_new = A_y + (Sy - A_y)*np.exp(-k_y*t)

    Sc_new = SAT_MAX - (SAT_MAX - Sc)*np.exp(-t/tau_c_uv)
    return Sy_new, Sc_new


def combined_uv_red(Sy, Sc, t):
    Sy_new = SAT_MAX - (SAT_MAX - Sy)*np.exp(-t/tau_y_uv)

    k_c = 1/tau_c_uv + 1/tau_c_red
    A_c = SAT_MAX*(1/tau_c_uv) / k_c
    Sc_new = A_c + (Sc - A_c)*np.exp(-k_c*t)
    return Sy_new, Sc_new

def get_sequence_with_hybrid(Sy_curr, Sc_curr, Sy_tgt, Sc_tgt):
    seq = get_sequence_sequential(Sy_curr, Sc_curr, Sy_tgt, Sc_tgt)

    if len(seq) == 2 and seq[0][0] == "UV" and seq[1][0] == "Blue":
        t_uv = seq[0][1]
        Sy1, Sc1 = combined_uv_blue(Sy_curr, Sc_curr, t_uv)
        t_blue = erase_time(Sy1, Sy_tgt, tau_y_blue)
        out = [("UV+Blue", round(t_uv, 2))]
        if t_blue > 0.01:
            out.append(("Blue", round(t_blue, 2)))
        return out

    if len(seq) == 2 and seq[0][0] == "UV" and seq[1][0] == "Red":
        t_uv = seq[0][1]
        Sy1, Sc1 = combined_uv_red(Sy_curr, Sc_curr, t_uv)
        t_red = erase_time(Sc1, Sc_tgt, tau_c_red)
        out = [("UV+Red", round(t_uv, 2))]
        if t_red > 0.01:
            out.append(("Red", round(t_red, 2)))
        return out

    return seq

# Step encoding
def encode_step(name, t):
    if name == "UV":
        return [1,0,0], [t,0,0]
    if name == "Red":
        return [0,1,0], [0,t,0]
    if name == "Blue":
        return [0,0,1], [0,0,t]
    if name == "UV+Blue":
        return [1,0,1], [t,0,t]
    if name == "UV+Red":
        return [1,1,0], [t,t,0]
    return [0,0,0], [0,0,0]

# Main function
def plan_from_saturations(Sy_curr, Sc_curr, Sy_tgt, Sc_tgt):
    steps = get_sequence_with_hybrid(Sy_curr, Sc_curr, Sy_tgt, Sc_tgt)

    lights = []
    times = []
    step_plan = []
    tokens = []

    for name, t in steps:
        L, T = encode_step(name, t)
        lights.append(L)
        times.append(T)
        step_plan.append((L, T))
        tokens.append(name)

    plan = " > ".join(tokens) if len(tokens) > 1 else f"{tokens[0]} only"
    closest_hex = color_from_saturations(Sy_tgt, Sc_tgt)
    return lights, times, plan, step_plan, closest_hex

def get_led_seq_from_rgb(hex_color_curr, hex_color_tgt):
    Y0, C0 = invert_color_to_saturations(hex_color_curr)
    Y1, C1 = invert_color_to_saturations(hex_color_tgt)
    return plan_from_saturations(Y0, C0, Y1, C1)

# Test runner
def run_test(label, Syc, Scc, Syt, Sct):
    rgb1 = color_from_saturations(Syc, Scc)
    rgb2 = color_from_saturations(Syt, Sct)
    lights, times, plan, step = plan_from_saturations(Syc, Scc, Syt, Sct)

    print(f"\n=== {label} ===")
    print(f"  {rgb1}  ({Syc:.2f}, {Scc:.2f}) → {rgb2} ({Syt:.2f}, {Sct:.2f})")
    print("  Plan     :", plan)
    print("  Lights   :", lights)
    print("  Times    :", times)
    print("  StepPlan :", step)