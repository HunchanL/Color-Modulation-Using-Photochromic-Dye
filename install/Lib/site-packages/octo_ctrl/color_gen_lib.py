import numpy as np
import time
from scipy.optimize import minimize
import colorsys

# def get_led_seq_from_rgb(hex_curr, hex_tgt, sat_gain=0.8, print_output=True):
#     """
#     Estimate LED activation commands to transition from current to target RGB (hex).
#     τ values correspond to time for ~63% saturation. Full visible (~95%) saturation 
#     emerges naturally from the exponential kinetics.
#     """

#     # ---- 1. Characteristic time constants (s) ----
#     tauC_UV, tauY_UV, tauC_red, tauY_blue = 25.8, 58.3, 55.0, 4.0
#     kUC, kUY, kRC, kBY = 1/tauC_UV, 1/tauY_UV, 1/tauC_red, 1/tauY_blue
#     epsF, satCap, desatCap = 1e-3, 0.95, 0.05

#     # ---- 2. Reference dye absorbance (fully saturated) ----
#     rgb_yellow = np.array([205, 207, 35]) / 255.0
#     rgb_cyan   = np.array([10, 15, 78]) / 255.0
#     to_absorb = lambda rgb: np.log(1.0 / np.maximum(rgb, 1e-6))
#     A_y, A_c = to_absorb(rgb_yellow), to_absorb(rgb_cyan)

#     # ---- 3. Calibrate absolute absorption to make #262D1F a mid-high saturation green ----
#     rgb_green_max = np.array([0x26, 0x2D, 0x1F]) / 255.0
#     A_green = np.log(1.0 / np.maximum(rgb_green_max, 1e-6))
#     scale = np.linalg.lstsq(np.column_stack([0.75*A_y, 0.75*A_c]), A_green, rcond=None)[0]
#     A_y *= scale[0];  A_c *= scale[1]
#     A_t = np.column_stack([A_y, A_c])

#     # ---- 4. HSV-based saturation scaling ----
#     def adjust_saturation(rgb, sat_gain=0.5):
#         h, s, v = colorsys.rgb_to_hsv(*rgb)
#         s = np.clip(s * sat_gain, 0, 1)
#         return np.clip(np.array(colorsys.hsv_to_rgb(h, s, v)), 0, 1)

#     # ---- 5. Convert between hex and dye fractions ----
#     def hex_to_YC(hex_str):
#         if hex_str.startswith('#'):
#             hex_str = hex_str[1:]
#         rgb = np.array([int(hex_str[i:i+2], 16) for i in (0,2,4)]) / 255.0
#         abs_meas = to_absorb(rgb)
#         YC = np.linalg.pinv(A_t) @ abs_meas
#         YC = np.clip(YC, desatCap, satCap)
#         return YC[0], YC[1]

#     def YC_to_rgb(Y, C):
#         """Convert dye fractions (Y,C) → RGB using Beer–Lambert model."""
#         A = Y*A_y + C*A_c
#         rgb = np.exp(-A)                   # removed opacity_factor scaling
#         rgb = np.clip(rgb, 0, 1)
#         rgb = rgb ** (1/2.2)               # gamma correction
#         rgb = adjust_saturation(rgb, sat_gain)
#         return rgb

#     # ---- 6. Input conversion ----
#     Y_curr, C_curr = hex_to_YC(hex_curr)
#     Y_tgt_raw, C_tgt_raw = hex_to_YC(hex_tgt)

#     # ---- 7. Project target into feasible gamut ----
#     rgb_tgt_meas = np.array([int(hex_tgt[i:i+2], 16) for i in (1,3,5)]) / 255.0 if hex_tgt.startswith("#") \
#                    else np.array([int(hex_tgt[i:i+2], 16) for i in (0,2,4)]) / 255.0
#     def err_gamut(YC):
#         rgb_pred = YC_to_rgb(YC[0], YC[1])
#         return np.linalg.norm(rgb_pred - rgb_tgt_meas)
#     Y0, C0 = np.clip([Y_tgt_raw, C_tgt_raw], desatCap, satCap)
#     res_proj = minimize(err_gamut, [Y0, C0], bounds=[(desatCap, satCap), (desatCap, satCap)])
#     Y_tgt, C_tgt = res_proj.x

#     # ---- 8. Closest achievable color ----
#     rgb_closest = YC_to_rgb(Y_tgt, C_tgt)
#     closest_hex = '#%02x%02x%02x' % tuple(int(255*x) for x in rgb_closest)

#     # ---- 9. Prepare kinetic variables ----
#     Y_curr, C_curr, Y_tgt, C_tgt = [
#         np.clip(v, desatCap, satCap) for v in (Y_curr, C_curr, Y_tgt, C_tgt)
#     ]
#     phiY0, phiYt = Y_curr, Y_tgt     # removed 0.5 scaling
#     phiC0, phiCt = C_curr, C_tgt

#     # ---- 10. Kinetic equations ----
#     def mix_time(phi0, phit, a, b):
#         phit = min(phit, 0.95)
#         phi_inf = a / (a + b)
#         phi_inf = np.clip(phi_inf, 0.05, 0.95)
#         if (phit - phi_inf)*(phi0 - phi_inf) > 0:
#             return max(0, (1/(a+b)) * np.log((phi0 - phi_inf)/max(phit - phi_inf, epsF)))
#         return 0

#     def uv_time(phi0, phit, k):
#         phit = min(phit, 0.95)
#         if phit > phi0 + epsF:
#             return max(0, (1/k) * np.log((1 - phi0) / max(1 - phit, epsF)))
#         return 0

#     def clear_time(phi0, phit, k):
#         phit = max(phit, 0.05)
#         if phit < phi0 - epsF:
#             return max(0, (1/k) * np.log(phi0 / max(phit, epsF)))
#         return 0

#     # ---- 11. LED sequence logic ----
#     dY, dC = Y_tgt - Y_curr, C_tgt - C_curr
#     fC = C_tgt / (C_tgt + Y_tgt + epsF)
#     is_yellow_column = (fC < 0.15)
#     t_uv = t_red = t_blue = 0; plan = "none"; lights = (0,0,0)

#     if dC >= -epsF and dY >= -epsF:
#         if is_yellow_column:
#             C_inf = kUC / (kUC + kRC)
#             tY = uv_time(phiY0, phiYt, kUY)
#             if phiCt >= C_inf - 1e-12:
#                 tC = mix_time(phiC0, phiCt, kUC, kRC)
#                 t_uv = t_red = max(tY, tC); plan = "UV + Red"; lights = (1,1,0)
#             else:
#                 t_uv = tY
#                 C_after = C_inf + (phiC0 - C_inf)*np.exp(-(kUC+kRC)*t_uv)
#                 t_red = clear_time(C_after, phiCt, kRC)
#                 plan = "UV + Red (simultaneous) > Red"; lights = (1,1,0)
#         else:
#             tC = uv_time(phiC0, phiCt, kUC)
#             tY = uv_time(phiY0, phiYt, kUY)
#             t_uv = max(tC, tY); plan = "UV"; lights = (1,0,0)
#     elif dC <= epsF and dY <= epsF:
#         t_red  = clear_time(phiC0, phiCt, kRC)
#         t_blue = clear_time(phiY0, phiYt, kBY)
#         plan = "Red + Blue"; lights = (0,1,1)
#     elif dY > epsF and dC < -epsF:
#         C_inf = kUC / (kUC + kRC)
#         tY = uv_time(phiY0, phiYt, kUY)
#         if phiCt >= C_inf - 1e-12:
#             tC = mix_time(phiC0, phiCt, kUC, kRC)
#             t_uv = t_red = max(tY, tC); plan = "UV + Red"; lights = (1,1,0)
#         else:
#             t_uv = tY
#             C_after = C_inf + (phiC0 - C_inf)*np.exp(-(kUC+kRC)*t_uv)
#             t_red = clear_time(C_after, phiCt, kRC)
#             plan = "UV + Red (simultaneous) > Red"; lights = (1,1,0)
#     elif dY < -epsF and dC > epsF:
#         Y_inf = kUY / (kUY + kBY)
#         tC = uv_time(phiC0, phiCt, kUC)
#         if phiYt >= Y_inf - 1e-12:
#             tY = mix_time(phiY0, phiYt, kUY, kBY)
#             t_uv = t_blue = max(tC, tY); plan = "UV + Blue"; lights = (1,0,1)
#         else:
#             t_uv = tC
#             Y_after = Y_inf + (phiY0 - Y_inf)*np.exp(-(kUY+kBY)*t_uv)
#             t_blue = clear_time(Y_after, phiYt, kBY)
#             plan = "UV + Blue (simultaneous) > Blue"; lights = (1,0,1)

#     # ---- 12. Output ----
#     times = (round(t_uv,1), round(t_red,1), round(t_blue,1))
#     step_plan = []
#     if plan == "UV":
#         step_plan.append(((1,0,0), times[0]))
#     elif plan == "Red + Blue":
#         step_plan.append(((0,1,1), round(max(t_red,t_blue),1)))
#     elif plan == "UV + Red":
#         step_plan.append(((1,1,0), times[0]))
#     elif plan == "UV + Red (simultaneous) > Red":
#         step_plan.append(((1,1,0), times[0]))
#         step_plan.append(((0,1,0), times[1]))
#     elif plan == "UV + Blue":
#         step_plan.append(((1,0,1), times[0]))
#     elif plan == "UV + Blue (simultaneous) > Blue":
#         step_plan.append(((1,0,1), times[0]))
#         step_plan.append(((0,0,1), times[2]))
#     else:
#         step_plan.append(((0,0,0), 0.0))

#     if print_output:
#         print(f"\nPlan: {plan}")
#         print(f"Closest achievable color: {closest_hex}")
#         print(f"Target dye fractions: Y={Y_tgt:.2f}, C={C_tgt:.2f}")
#         for i, (ls, t) in enumerate(step_plan):
#             print(f"Step {i+1}: UV={ls[0]} Red={ls[1]} Blue={ls[2]} for {t:.1f}s")

#     return lights, times, plan, step_plan, closest_hex


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