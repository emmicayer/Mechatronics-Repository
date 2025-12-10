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
        if s_mot_cmd.get() != 1:
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
