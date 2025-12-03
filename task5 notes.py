   # ady = pyb.ADC(pyb.Pin('pin number'))              # Include these two lines to read battery voltage
    #v_bat = adc.read()*(3.3/4095)*((R1+R2)/R2)



# Friday distance gone: 0.8 m
DC u=[0.000,0.000] y=[sL=-0.0855, sR=0.0855, th=5.070, gz=0.000]
bump mask=0xC
DC u=[0.381,2.506] y=[sL=0.2091, sR=0.3729, th=5.053, gz=-0.077]





# Tuesday: working on task 5, getting errors in window running with PuTTY


# NOTES
import pyb
from pyb import UART
from ulab import numpy as np

# Define constants, arrays, and observer gain L in LTI form
w   = 0.141 # Track width, distance between wheels [m]
r   = 0.035
K   = (250.0 * 2.0 * 3.14159265 / 60.0) / 4.5   # Motor gain [rad/(v*s)]
tau = 0.1                                       # Motor time constant [s]

A = np.array([[ -1/tau, 0.0, 0.0, 0.0],
              [ 0.0, -1.0/tau, 0.0, 0.0],
              [ r/2, r/2, 0.0 , 0.0],
              [ -r/w, r/w, 0.0, 0.0]])

B = np.array([[ K/tau, 0.0],
              [ 0.0, K/tau],
              [ 0.0, 0.0],
              [ 0.0, 0.0]])


C = np.array([[ 0.0, 0.0, 1.0, -w/2],            
              [ 0.0, 0.0, 1.0, w/2],              
              [ 0.0, 0.0, 0.0, 1.0],  
              [ -r/w, r/w, 0.0, 0.0]  
              ])

D = np.zeros((4,2))

# Heading theta ≈ (x_R - x_L)/(w) = (x_R - x_L)/w
# theta_dot ≈ (v_R - v_L)/w

# Functions

def columnize_vector(v, n_expected):            # v is the input argument, n is number of rows expected
    out = np.zeros((n_expected, 1))

    if v is None:
            return out
    if isinstance(v, (int, float)):
        out[0, 0] = float(v)
        return out
    # Try to iterate; if not iterable, fall back to scalar cast
    try:
        i = 0
        for val in v:
            if i >= n_expected:
                break
            try:
                out[i, 0] = float(val)
            except Exception:
                # non-numeric element -> leave as 0.0
                pass
            i += 1
        return out
    except Exception:
        # Not iterable; last-chance scalar cast
        try:
            out[0, 0] = float(v)
        except Exception:
            pass
        return out
    

def continuous_time_dynamics(x, u):     # x current state estimate, u current input vector 
    x = columnize_vector(x, 4)
    u = columnize_vector(u, 2)
    Ax = np.dot(A, x)
    Bu = np.dot(B, u)
    return Ax + Bu

def output_equation(x, u=None):
    x = columnize_vector(x, 4)
    y = np.dot(C, x)
    if u is not None:
        u = columnize_vector(u, 2)
        y = y + np.dot(D, u)
    return y

def RK4_solver(x, u, dt):
    k1 = continuous_time_dynamics(x, u)
    k2 = continuous_time_dynamics(x + 0.5*dt*k1, u)
    k3 = continuous_time_dynamics(x + 0.5*dt*k2, u)
    k4 = continuous_time_dynamics(x + dt*k3, u)
    return x + (dt/ 6.0)*(k1 + 2*k2 + 2*k3 + k4)


# Generator state estimation task
def StateEstimation(shares):
    s_mot_cmd, s_mot_eff_L, s_mot_eff_R, s_new_setpoint_L, s_new_setpoint_R, s_xhat, s_yhat, q_u, q_y = shares
    state = 0 
    BT = UART(1, 115200)

    xhat = np.zeros((4,1))      # 4x1 estimated state vector, starts at zeros
    u = np.zeros((2,1))         # 2x1 input vector (left and right motor voltages), starts at zeros
    y_meas = np.zeros((4,1))    # 4x1 measurement vector (enocders and IMU), starts at zeros
    last_t = pyb.micros()                       # Get current timer to timestamp when iteration starts, used to compute dt
    print("[StateEst] ready")
    next_print_time = 0

    yield 0
    while True:
        now_t = pyb.micros()      # Reads current time
        dt_us = (now_t - last_t) & 0xFFFFFFFF   # Masks dt to 32 bits so subtraction works with wrap around
        last_t = now_t
        if dt_us <= 0:              # Safety if statement if the time wraps to negative
            dt_us = 10000
        dt = float(dt_us)*1e-6      # Converts to seconds (RK4 wants seconds)

        # Get most recent u and y values
        u_buf = []
        try:
            while q_u.any():
                u_buf.append(float(q_u.get()))
                if len(u_buf) >= 2:
                    u = columnize_vector([u_buf[-2], u_buf[-1]], 2)
        except AttributeError:
            while not q_u.empty():
                u_buf.append(float(q_u.get()))
                if len(u_buf) >= 2:
                    u = columnize_vector([u_buf[-2], u_buf[-1]], 2)

        # --- read newest COMPLETE quad from q_y ---
        y_buf = []
        try:
            while q_y.any():
                y_buf.append(float(q_y.get()))
                if len(y_buf) >= 4:
                    y_meas = columnize_vector([y_buf[-4], y_buf[-3], y_buf[-2], y_buf[-1]], 4)
        except AttributeError:
            while not q_y.empty():
                y_buf.append(float(q_y.get()))
                if len(y_buf) >= 4:
                    y_meas = columnize_vector([y_buf[-4], y_buf[-3], y_buf[-2], y_buf[-1]], 4)

        # try:
        #     if hasattr(q_u, "any"):     # hasattr(object, attribute name), checks if an object has an attribute name
        #         while q_u.any():        # Loops until queue is emptied if q_u has a value. u will hold newest input
        #             u = columnize_vector(q_u.get(), 2)  # Converts whatever came from queue to 2x1 column vector
        #     else:
        #         if not q_u.empty():     # If empty, return True and give last value
        #             u = columnize_vector(q_u.get(), 2)
        # except Exception:
        #     pass

        # try:
        #     if hasattr(q_y, "any"):     # Same for y
        #         while q_y.any():
        #             y_meas = columnize_vector(q_y.get(), 4)
        #     else:
        #         if not q_y.empty():
        #             y_meas = columnize_vector(q_y.get(), 4)
        # except Exception:
        #     pass

        xhat = RK4_solver(xhat, u, dt)    # Uses RK4 solver 
        yhat = output_equation(xhat)                      
    

        theta_hat = float(yhat[2,0])      # Heading estimate
        s_yhat.put(theta_hat)

        if (s_mot_cmd.get() == 1.0) and (now_t >= next_print_time):
            e = y_meas - yhat
            print("θ̂ = {:.3f} rad".format(theta_hat))
            print("SE dt={:.3f} ms | e[sL]={:+.4f}  e[sR]={:+.4f}  e[θ]={:+.4f}  e[θ̇]={:+.4f}"
                .format(dt*1e3, float(e[0,0]), float(e[1,0]), float(e[2,0]), float(e[3,0])))
            next_print_time = now_t + 200000  # 200 ms

        # try:
        #     sdot_hat = float((r*0.5)*(xhat[0,0] + xhat[1,0]))
        #     s_xhat.put(sdot_hat)
        # except Exception:
        #     pass

        # if now_t >= next_print_time:
        #     e = y_meas - yhat
        #     try:
        #         next_print_time = now_t + 50000     # Add 50000 ms to the next print time
        #         print("SE dt={:.3f} ms | e[sL]={:+.4f}  e[sR]={:+.4f}  e[θ]={:+.4f}  e[θ̇]={:+.4f}".format(dt*1e3, float(e[0,0]), float(e[1,0]), float(e[2,0]), float(e[3,0])))
        #     except Exception:
        #         pass



        #   # Bluetooth steaming of data collection
        # line = "SE dt={:.3f} ms | e[sL]={:+.4f}  e[sR]={:+.4f}  e[θ]={:+.4f}  e[θ̇]={:+.4f}".format(dt*1e3, float(e[0,0]), float(e[1,0]), float(e[2,0]), float(e[3,0]))
        # try:
        #     BT.write(line.encode())
        # except:
        #     pass

        # if s_mot_cmd.get() == 0.0:
        #     state = 1
        


        yield 0



# NOTES













# task5_State_Estimation.py
# State estimator task for Romi using RK4 and Luenberger observer form.
# Micropython + ulab compatible (no @ operator; uses np.dot)
#
# Inputs (via queues):
#   q_u: latest input vector u = [V_L, V_R] (2x1)
#   q_y: latest measurement vector y = [x_L, x_R, theta, theta_dot] (4x1)
# Outputs (via shares):
#   s_xhat: estimated state vector xhat (4x1)
#   s_yhat: estimated output vector yhat (4x1)
#
# The observer implements: xdot = (A-LC) x + (B-LD) u + L y
# with yhat = C x + D u
#
# Author: generated with GPT-5 Thinking for ME405-style framework

import pyb
import ulab.numpy as np

# ---------------------------
# Utility: columnize vectors
# ---------------------------
def as_col(v, n_expected):
    """Return v as an (n,1) ulab column vector. Accepts list/tuple/np.ndarray/scalar."""
    if isinstance(v, (int, float)):
        vv = np.array([[float(v)]], dtype=np.float)
    else:
        # Flatten and cast to float
        try:
            vv = np.array(v, dtype=np.float)
        except Exception:
            # Fallback: wrap in list
            vv = np.array([float(v)], dtype=np.float)
    # Make 1-D into column
    if vv.ndim == 1:
        vv = vv.reshape((vv.shape[0], 1))
    # Sanity pad/trim if needed
    n = vv.shape[0]
    if n < n_expected:
        pad = np.zeros((n_expected - n, 1), dtype=np.float)
        vv = np.vstack((vv, pad))
    elif n > n_expected:
        vv = vv[:n_expected, :]
    return vv

# ---------------------------------------------
# Default model parameters and system matrices
# ---------------------------------------------
# These are placeholders; tune using your HW 0x03 model.
# State x = [x_L, x_R, v_L, v_R]^T
# Input u = [V_L, V_R]^T
# Output y = [x_L, x_R, theta, theta_dot]^T
#
# Geometry (meters); adjust to your robot


# Simple “motor -> wheel” first-order dynamics as a starting point
# xdot = [ v_L, v_R, -a*v_L + b*V_L, -a*v_R + b*V_R ]
a = 5.0   # [1/s] viscous-like damping (tune)
b = 50.0  # [m/(s*V)] input gain (tune)

A = np.array([[ 0.0, 0.0, 1.0, 0.0],
              [ 0.0, 0.0, 0.0, 1.0],
              [ 0.0, 0.0, -a , 0.0],
              [ 0.0, 0.0, 0.0, -a ]], dtype=np.float)

B = np.array([[ 0.0, 0.0],
              [ 0.0, 0.0],
              [  b , 0.0],
              [ 0.0,  b ]], dtype=np.float)

# Heading theta ≈ (x_R - x_L)/(2*HALF_BASE) = (x_R - x_L)/TRACK_WIDTH
# theta_dot ≈ (v_R - v_L)/TRACK_WIDTH
C = np.array([[ 1.0, 0.0, 0.0, 0.0],              # x_L
              [ 0.0, 1.0, 0.0, 0.0],              # x_R
              [-1.0/TRACK_WIDTH, 1.0/TRACK_WIDTH, 0.0, 0.0],  # theta
              [ 0.0, 0.0, -1.0/TRACK_WIDTH, 1.0/TRACK_WIDTH]  # theta_dot
              ], dtype=np.float)

D = np.zeros((4,2), dtype=np.float)

# Observer gain L (4x4) – start conservative; tune later
L = np.array([[  8.0,   0.0,   0.0,   0.0],
              [  0.0,   8.0,   0.0,   0.0],
              [  0.0,   0.0,  15.0,   0.0],
              [  0.0,   0.0,   0.0,  15.0]], dtype=np.float)

# Precompute F=(A-LC), G=(B-LD) to reduce runtime multiplications
F = A - np.dot(L, C)
G = B - np.dot(L, D)  # D is zero here but keep it general

# --------------------------------
# Continuous-time model functions
# --------------------------------
def f_cont(x, u, y_meas):
    """Observer-augmented dynamics: xdot = F x + G u + L y_meas"""
    # Ensure shapes
    x = as_col(x, 4)
    u = as_col(u, 2)
    y = as_col(y_meas, 4)
    # Compute
    Fx = np.dot(F, x)
    Gu = np.dot(G, u)
    Ly = np.dot(L, y)
    return Fx + Gu + Ly

def h_out(x, u=None):
    """Output map yhat = C x + D u (u optional; D is zero for us)."""
    x = as_col(x, 4)
    y = np.dot(C, x)
    if u is not None:
        u = as_col(u, 2)
        y = y + np.dot(D, u)
    return y

# ----------------------
# RK4 single-step solver
# ----------------------
def rk4_step(x, u, y_meas, dt):
    x = as_col(x, 4)
    u = as_col(u, 2)
    y = as_col(y_meas, 4)

    k1 = f_cont(x,           u, y)
    k2 = f_cont(x + 0.5*dt*k1, u, y)
    k3 = f_cont(x + 0.5*dt*k2, u, y)
    k4 = f_cont(x +     dt*k3, u, y)
    return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)

# --------------------------------------
# Generator Task: State Estimation (LTI)
# --------------------------------------
def Task_StateEstimation(shares):
    """
    Cooperative task.
    shares = (s_xhat, s_yhat, q_u, q_y) where:
        - s_xhat, s_yhat are Share-like objects with .put(val)
        - q_u, q_y are Queue-like objects with .any()/.empty()/.get()
    Task period should be ~10 ms (100 Hz) or faster than plant poles.
    """
    # Unpack user-provided shares/queues
    s_xhat, s_yhat, q_u, q_y = shares

    # Initialize estimate and last I/O
    xhat = np.zeros((4,1), dtype=np.float)
    u    = np.zeros((2,1), dtype=np.float)
    y_meas = np.zeros((4,1), dtype=np.float)

    # Timekeeping
    last_t = pyb.micros()  # microseconds

    # Optional: warmup to avoid huge first dt
    yield 0

    while True:
        # --- timing / dt ---
        now = pyb.micros()
        dt_us = (now - last_t) & 0xFFFFFFFF  # wrap-safe
        last_t = now
        if dt_us <= 0:
            dt_us = 10000  # fallback 10 ms
        dt = float(dt_us) * 1e-6

        # --- inputs ---
        # Get most recent available u and y (non-blocking)
        # If multiple pending, consume the latest one
        if hasattr(q_u, "any"):
            while q_u.any():
                u = as_col(q_u.get(), 2)
        else:
            if not q_u.empty():
                u = as_col(q_u.get(), 2)

        if hasattr(q_y, "any"):
            while q_y.any():
                y_meas = as_col(q_y.get(), 4)
        else:
            if not q_y.empty():
                y_meas = as_col(q_y.get(), 4)

        # --- observer RK4 step ---
        xhat = rk4_step(xhat, u, y_meas, dt)

        # --- outputs ---
        yhat = h_out(xhat)  # D*u is zero here
        # Publish to shares (most frameworks copy references; keep small arrays)
        if hasattr(s_xhat, "put"):
            s_xhat.put(xhat)
        if hasattr(s_yhat, "put"):
            s_yhat.put(yhat)

        # --- debug (optional): send occasionally over USB serial ---
        # vcp = pyb.USB_VCP()
        # if vcp.isconnected():
        #     vcp.write(("y_meas theta={:+.3f}  yhat theta={:+.3f}\r\n"
        #         .format(float(y_meas[2,0]), float(yhat[2,0]))).encode())

        # Yield to scheduler
        yield 0

# -----------------------------
# Helper: convenient constructor
# -----------------------------
def make_state_task(cotask, task_share, period_ms=10, priority=2):
    """
    Convenience function to create and append the state estimation task
    assuming ME405-style cotask/task_share modules.
    Returns the created task and the created shares/queues.
    """
    # Create shares/queues
    s_xhat = task_share.Share(type_code='f', thread_protect=False, name="xhat")
    s_yhat = task_share.Share(type_code='f', thread_protect=False, name="yhat")
    q_u    = task_share.Queue(type_code='f', size=8, thread_protect=False, name="u")
    q_y    = task_share.Queue(type_code='f', size=8, thread_protect=False, name="y")

    # Create task
    t = cotask.Task(Task_StateEstimation,
                    name="StateEst",
                    priority=priority,
                    period=period_ms,
                    profile=False,
                    trace=False,
                    shares=(s_xhat, s_yhat, q_u, q_y))

    cotask.task_list.append(t)
    return t, (s_xhat, s_yhat, q_u, q_y)

# -----------------------------
# Tuning hooks (optional)
# -----------------------------
def set_matrices(A_new=None, B_new=None, C_new=None, D_new=None, L_new=None):
    """
    Update global matrices at runtime (e.g., for tuning).
    Recomputes F and G accordingly.
    """
    global A, B, C, D, L, F, G
    if A_new is not None: A = np.array(A_new, dtype=np.float)
    if B_new is not None: B = np.array(B_new, dtype=np.float)
    if C_new is not None: C = np.array(C_new, dtype=np.float)
    if D_new is not None: D = np.array(D_new, dtype=np.float)
    if L_new is not None: L = np.array(L_new, dtype=np.float)
    F = A - np.dot(L, C)
    G = B - np.dot(L, D)
