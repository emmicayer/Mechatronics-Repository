## @file task5_StateEstimation_DT.py
#  @brief Discrete-time state estimation task using a linear observer model to
#  estimate forward velocity, yaw rate, and heading from encoder and IMU data.
#
#  This task implements a discrete observer, updated at each scheduler
#  cycle, using precomputed Ad and Bd matrices derived from a MATLAB c2d model.
#  It reads:
#     • Motor input voltages (u) from the input queue
#     • Wheel displacements and IMU measurements (y) from the measurement queue
#
#  Using these, it computes:
#     • Estimated forward velocity
#     • Estimated yaw rate
#     • Integrated heading (theta)
#  These are published to shared variables for use by navigation, diagnostics,
#  and higher-level control.
#
#  The task gracefully handles missing data, automatically computes sampling
#  intervals using `micros()`, and prints debug information periodically. It
#  runs only while the robot is active, yielding often to coexist with other
#  scheduler tasks.


import pyb
from ulab import numpy as np

# Parameters
r = 0.035   # Wheel radius [m]
w = 0.141   # Track width [m]

# Matrices from MATLAB
A_D = np.array([
    [ 0.0011,  0.0011,  0.0093,  0.0   ],
    [ 0.0011,  0.0011,  0.0093,  0.0   ],
    [-0.0,    -0.0,     0.0026,  0.0   ],
    [-0.0,    -0.0,     0.0,     0.0   ],
])

B_D = np.array([
    [ 0.0403,  0.0319, -0.0047, -0.0047, -0.0000, -2.0114],
    [ 0.0319,  0.0403, -0.0047, -0.0047,  0.0000,  2.0114],
    [ 0.0021,  0.0021,  0.4987,  0.4987, -0.0000,  0.0000],
    [ 0.0000,  0.0000, -0.0071,  0.0071,  0.0001,  0.0001],
])

# Du is inputs, Dy is measurements
B_Du = B_D[:, 0:2]
B_Dy = B_D[:, 2:6]

# Column vector helper functions
def _column4(): return np.zeros((4, 1))
def _column2(): return np.zeros((2, 1))

# State estimation task
def StateEstimation(shares):
    (s_mot_cmd, s_mot_eff_L, s_mot_eff_R, s_new_setpoint_L, s_new_setpoint_R,
     s_xhat, s_yhat, q_u, q_y) = shares

    # Initialize state, input, helper variables
    xhat = _column4()   # Initial state estimate
    u    = _column2()   # Control input

    theta_hat = 0.0 # Integrated heading estimate
    prev_sL = None  # previous left wheel displacement
    prev_sR = None  # Previous right wheel displacement

    Ts_nom = 0.020  # Sample period [20 ms]
    last_t = pyb.micros()
    next_print = 0  # For debugging

    try:
        print("[StateEst-DT|MATLAB] ready (Ts=20 ms, Ad/Bd from c2d)")
    except Exception:
        pass

    yield 0


    while True: # Only run if motor is on
        if s_mot_cmd.get() != 1.0:
            yield 0
            continue

        # Compute time step
        now_t = pyb.micros()
        dt = float(((now_t - last_t) & 0xFFFFFFFF)) * 1e-6
        last_t = now_t
        if dt <= 0.0 or dt > 0.2:
            dt = Ts_nom

        # get most recent motor imput command
        try:
            buf_u = []
            if hasattr(q_u, "any"):
                while q_u.any():
                    buf_u.append(float(q_u.get()))
            else:
                while not q_u.empty():
                    buf_u.append(float(q_u.get()))
            # Use most recent u, [vL, vR]
            if len(buf_u) >= 2:
                u[0, 0] = buf_u[-2]
                u[1, 0] = buf_u[-1]
        except Exception:
            pass

        # Get last measurement data
        sL_m = None; sR_m = None; theta_m = None; gz = None
        try:
            buf_y = []
            if hasattr(q_y, "any"):
                while q_y.any():
                    buf_y.append(float(q_y.get()))
            else:
                while not q_y.empty():
                    buf_y.append(float(q_y.get()))
            if len(buf_y) >= 4: # [sL_m, sR_m, theta, gz]
                sL_m   = buf_y[-4]
                sR_m   = buf_y[-3]
                theta_m= buf_y[-2]
                gz     = buf_y[-1]
        except Exception:
            pass

        # Skip if no data yet
        if sL_m is None or sR_m is None or gz is None:
            yield 0
            continue

        # Find wheel linear velocities from displacement
        if prev_sL is None: # Assume zero velocities if no previous data
            vL_meas = 0.0
            vR_meas = 0.0
        else:
            vL_meas = (sL_m - prev_sL) / dt
            vR_meas = (sR_m - prev_sR) / dt
        prev_sL, prev_sR = sL_m, sR_m

        # Compute yaw rate from wheel speeds 
        yaw_rate_wheels = (-vL_meas + vR_meas) / w

        # Measurement vector from obsever
        y_vec = np.array([[vL_meas],
                          [vR_meas],
                          [gz],
                          [yaw_rate_wheels]])

        # discrete observer update
        xhat = np.dot(A_D, xhat) + np.dot(B_Du, u) + np.dot(B_Dy, y_vec)

        # Get outputs and publish them
        v_hat = float(xhat[2, 0])           # Forward velocity estimate
        yaw_rate_hat = float(xhat[3, 0])    # Yaw rate estimate
        theta_hat += yaw_rate_hat * dt      # Integrate yaw rate to get heading

        # Publish estimates to shares and queues
        try:
            s_xhat.put(v_hat)
        except Exception:
            pass
        try:
            s_yhat.put(theta_hat)
        except Exception:
            pass

        # Debug output
        if now_t >= next_print:
            next_print = now_t + 2000000  # 2 seconds
            try:
                print(("SE-DT|MAT dt={:.2f}ms | vL_meas={:+.3f} vR_meas={:+.3f} "
                       "gz={:+.3f} yaw_w={:+.3f} | v_hat={:+.3f} yaw_hat={:+.3f} theta_hat={:+.2f}")
                      .format(1e3*dt, vL_meas, vR_meas, gz, yaw_rate_wheels,
                              v_hat, yaw_rate_hat, theta_hat))
            except Exception:
                pass

        yield 0





# import pyb
# from ulab import numpy as np

# # Define constants, arrays, and observer gain L in LTI form
# w   = 0.141  # track width [m]
# r   = 0.035  # wheel radius [m]
# K   = (250.0 * 2.0 * 3.14159265 / 60.0) / 4.5   # motor gain [rad/(V*s)]
# tau = 0.1                                       # motor time constant [s]

# # Continuous-time model 
# A_D = np.array([
#     [ 0.0011,  0.0011,  0.0093,  0.0   ],
#     [ 0.0011,  0.0011,  0.0093,  0.0   ],
#     [-0.0,    -0.0,     0.0026,  0.0   ],
#     [-0.0,    -0.0,     0.0,     0.0   ],
# ])

# B_D = np.array([
#     [ 0.0403,  0.0319, -0.0047, -0.0047, -0.0000, -2.0114],
#     [ 0.0319,  0.0403, -0.0047, -0.0047,  0.0000,  2.0114],
#     [ 0.0021,  0.0021,  0.4987,  0.4987, -0.0000,  0.0000],
#     [ 0.0000,  0.0000, -0.0071,  0.0071,  0.0001,  0.0001],
# ])

# B_Du = B_D[:, 0:2]
# B_Dy = B_D[:, 2:6]

# Ld = np.array([[  6685.0,   6685.0,   0.0,   -13844.0],
#                [  6685.0,   6685.0,   0.0,   13844.0],
#                [  295.0,   295.0,   0.5,   0.0],
#                [  -49.0,   49.0,   1.0,   1.0]])

# # Discretization
# def _euler_discretize(A, B, dt):
#     """Simple forward-Euler discretization (fast on MCU; replace with exact (c2d) numbers if desired)."""
#     I = np.eye(A.shape[0])
#     Ad = I + A * dt
#     Bd = B * dt
#     return Ad, Bd

# # Create column vectors
# def _column4():
#     return np.zeros((4, 1))

# def _column2():
#     return np.zeros((2, 1))

# # Ensure x is n x 1 ulab column vector
# def _ensure_vec(x, n):
#     x = np.array(x)
#     if x.shape == (n, ):
#         x = x.reshape((n, 1))
#     elif x.shape == (1, n):
#         x = x.T
#     return x

# # Diescrete time state estimation
# def StateEstimation(shares):
#     s_mot_cmd, s_mot_eff_L, s_mot_eff_R, s_new_setpoint_L, s_new_setpoint_R, s_xhat, s_yhat, q_u, q_y = shares

#     # State estimate x̂ and input/measurement vectors, initial values
#     xhat = _column4()          # [omega_L, omega_R, v, yaw_rate]
#     u    = _column2()          # [vL, vR]
#     y_meas = _column4()        # [sL_m, sR_m, theta, gz]

#     # Integrator states used to form y-hat (so outputs match your sensors)
#     sL_hat = 0.0
#     sR_hat = 0.0
#     theta_hat = 0.0

#     # Timing
#     last_t = pyb.micros()
#     next_print = 0

#     # If you want to lock the estimator to a fixed Ts (e.g., 20 ms from main scheduler), set this:
#     FIXED_TS = None  # e.g., 0.020  # seconds; leave None to compute from micros()

#     # Startup message
#     try:
#         print("[StateEst-DT] ready")
#     except Exception:
#         pass

#     # Initial yield required by the scheduler
#     yield 0

#     while True:
#         # Only run when motors are enabled 
#         if s_mot_cmd.get() != 1.0:
#             yield 0
#             continue

#         now_t = pyb.micros()
#         dt = FIXED_TS if FIXED_TS is not None else float(((now_t - last_t) & 0xFFFFFFFF)) * 1e-6
#         last_t = now_t
#         if dt <= 0.0 or dt > 0.2:  # Avoid jump in time
#             dt = 0.02

#         # Discretize quickly (Euler). Replace with precomputed Ad,Bd from c2d if you prefer.
#         Ad, Bd = _euler_discretize(A, B, dt)

#         # --- Pull the latest complete input pair from q_u ---
#         try:
#             buf_u = []
#             if hasattr(q_u, "any"):
#                 while q_u.any():
#                     buf_u.append(float(q_u.get()))
#             else:
#                 while not q_u.empty():
#                     buf_u.append(float(q_u.get()))
#             if len(buf_u) >= 2:
#                 u[0, 0] = buf_u[-2]
#                 u[1, 0] = buf_u[-1]
#         except Exception:
#             pass

#         # --- Pull the latest complete measurement quadruple from q_y ---
#         try:
#             buf_y = []
#             if hasattr(q_y, "any"):
#                 while q_y.any():
#                     buf_y.append(float(q_y.get()))
#             else:
#                 while not q_y.empty():
#                     buf_y.append(float(q_y.get()))
#             if len(buf_y) >= 4:
#                 # Order assumed from DataCollect: [sL_m, sR_m, theta, gz]
#                 y_meas[0, 0] = buf_y[-4]
#                 y_meas[1, 0] = buf_y[-3]
#                 y_meas[2, 0] = buf_y[-2]
#                 y_meas[3, 0] = buf_y[-1]
#         except Exception:
#             pass

#         # --- Form current y-hat from estimator's internal integrators and x̂ ---
#         # yaw_rate_hat comes from x̂[3,0]; sL_hat/sR_hat/theta_hat are integrated below.
#         yhat = _column4()
#         yhat[0, 0] = sL_hat
#         yhat[1, 0] = sR_hat
#         yhat[2, 0] = theta_hat
#         yhat[3, 0] = float(xhat[3, 0])  # yaw_rate_hat

#         # --- Innovation (measurement residual) ---
#         e = y_meas - yhat

#         # --- Discrete observer update ---
#         xhat = np.dot(Ad, xhat) + np.dot(Bd, u) + np.dot(Ld, e)

#         # --- Update output integrators using *new* state estimate ---
#         omega_L_hat = float(xhat[0, 0])
#         omega_R_hat = float(xhat[1, 0])
#         yaw_rate_hat = float(xhat[3, 0])

#         sL_hat += r * omega_L_hat * dt
#         sR_hat += r * omega_R_hat * dt
#         theta_hat += yaw_rate_hat * dt

#         # --- Publish convenient scalars for the rest of your system ---
#         # heading estimate (for your s_yhat consumer)
#         try:
#             s_yhat.put(theta_hat)
#         except Exception:
#             pass

#         # forward speed estimate (m/s): sdot_hat = r*(omega_L + omega_R)/2
#         try:
#             sdot_hat = 0.5 * r * (omega_L_hat + omega_R_hat)
#             s_xhat.put(sdot_hat)
#         except Exception:
#             pass

#         # --- Optional debug print (every ~2 s) ---
#         if now_t >= next_print:
#             next_print = now_t + 2_000_000
#             try:
#                 print("SE-DT dt={:.3f} ms | e=[{:+.4f} {:+.4f} {:+.4f} {:+.4f}] | theta_hat={:+.3f} rad | sdot_hat={:+.3f} m/s"
#                       .format(1e3*dt,
#                               float(e[0,0]), float(e[1,0]), float(e[2,0]), float(e[3,0]),
#                               theta_hat, sdot_hat))
#             except Exception:
#                 pass

#         yield 0
