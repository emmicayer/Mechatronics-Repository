import pyb
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
    have_u = False              # Flags
    have_y = False
    heading_initialized = False

    xhat = np.zeros((4,1))      # 4x1 estimated state vector, starts at zeros
    u = np.zeros((2,1))         # 2x1 input vector (left and right motor voltages), starts at zeros
    y_meas = np.zeros((4,1))    # 4x1 measurement vector (enocders and IMU), starts at zeros
    last_t = pyb.micros()                       # Get current timer to timestamp when iteration starts, used to compute dt
    print("[StateEst] ready")
    next_print_time = 0

    yield 0
    while True:
        if s_mot_cmd.get() != 1.0:
            yield 0
            continue
        
           # One-time seed: grab one measurement set to initialize theta_hat and baselines
        if not heading_initialized:
            tmp = []
            if hasattr(q_y, "any"):
                while q_y.any():
                    tmp.append(float(q_y.get()))
            else:
                while not q_y.empty():
                    tmp.append(float(q_y.get()))
            if len(tmp) >= 4:
                # tmp order is [sL_m, sR_m, theta, gz]
                prev_sL = tmp[-4]
                prev_sR = tmp[-3]
                theta_hat = tmp[-2]   # seed heading so SE-DT publishes your IMU yaw immediately
                seeded = True

        now_t = pyb.micros()      # Reads current time
        dt_us = (now_t - last_t) & 0xFFFFFFFF   # Masks dt to 32 bits so subtraction works with wrap around
        last_t = now_t
        if dt_us <= 0:              # Safety if statement if the time wraps to negative
            dt_us = 10000
        dt = float(dt_us)*1e-6      # Converts to seconds (RK4 wants seconds)

        # Get most recent u and y values
        try:
            us = []
            # if hasattr(q_u, "num_in"):
            #     while q_u.num_in() >= 2:
            #         u[0,0] = float(q_u.get())
            #         u[1,0] = float(q_u.get())
            # else:
            #     # Fallback if no num_in(): try a guarded read
            #     try:
            #         u0 = float(q_u.get()); u1 = float(q_u.get())
            #         u[0,0], u[1,0] = u0, u1
            #     except: pass
            
            if hasattr(q_u, "any"):
                while q_u.any():
                    us.append(float(q_u.get()))
            else:
                while not q_u.empty():
                    us.append(float(q_u.get()))
            if len(us) >= 2:
                # use the last complete pair (vL, vR)
                u[0, 0] = us[-2]
                u[1, 0] = us[-1]
                have_u = True
        except Exception:
            pass

        # --- Get most recent y = [sL_m, sR_m, theta, gz]^T ---
        try:
            ys = []
            # if hasattr(q_y, "num_in"):
            #     while q_y.num_in() >= 4:
            #         y_meas[0,0] = float(q_y.get())
            #         y_meas[1,0] = float(q_y.get())
            #         y_meas[2,0] = float(q_y.get())
            #         y_meas[3,0] = float(q_y.get())
            # else:
            #     try:
            #         y_meas[0,0] = float(q_y.get())
            #         y_meas[1,0] = float(q_y.get())
            #         y_meas[2,0] = float(q_y.get())
            #         y_meas[3,0] = float(q_y.get())
            #     except: pass
            
            
            if hasattr(q_y, "any"):
                while q_y.any():
                    ys.append(float(q_y.get()))
            else:
                while not q_y.empty():
                    ys.append(float(q_y.get()))
            if len(ys) >= 4:
                # use the last complete quadruple
                y_meas[0, 0] = ys[-4]   # sL_m
                y_meas[1, 0] = ys[-3]   # sR_m
                y_meas[2, 0] = ys[-2]   # theta
                y_meas[3, 0] = ys[-1]   # gz
                have_y = True
        except Exception:
            pass
    

        if not (have_u and have_y):
            # Queue visibility 
            try:
                if hasattr(q_u, "num_in") and hasattr(q_y, "num_in"):
                    print("SE waiting: q_u={}, q_y={}".format(q_u.num_in(), q_y.num_in()))
            except Exception:
                pass
            yield 0
            continue

        if hasattr(q_u, "num_in") and q_u.num_in() >= 2:
            u[0,0] = float(q_u.get()); u[1,0] = float(q_u.get()); have_u = True

        # outputs
        if hasattr(q_y, "num_in") and q_y.num_in() >= 4:
            y_meas[0,0] = float(q_y.get())
            y_meas[1,0] = float(q_y.get())
            y_meas[2,0] = float(q_y.get())
            y_meas[3,0] = float(q_y.get())
            have_y = True    
        # Debug
        # if (abs(u[0,0]) > 1e-9 or abs(u[1,0]) > 1e-9 or
        #     abs(y_meas[0,0]) > 1e-9 or abs(y_meas[1,0]) > 1e-9 or
        #     abs(y_meas[2,0]) > 1e-9 or abs(y_meas[3,0]) > 1e-9):
        #     try:
        #         print("DBG u=[{:.3f},{:.3f}] y=[sL={:.4f}, sR={:.4f}, th={:.3f}, gz={:.3f}]"
        #             .format(u[0,0], u[1,0], y_meas[0,0], y_meas[1,0], y_meas[2,0], y_meas[3,0]))
        #     except:
        #         pass
       
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
        e = y_meas - yhat

        theta_hat = float(yhat[2,0])      # Heading estimate
        # print("θ̂ = {:.3f} rad".format(theta_hat))
        s_yhat.put(theta_hat)
        try:
            sdot_hat = float((r*0.5)*(xhat[0,0] + xhat[1,0]))
            s_xhat.put(sdot_hat)
        except Exception:
            pass

        if now_t >= next_print_time:
            e = y_meas - yhat
            try:
                next_print_time = now_t + 2000000     # Add 2000 ms to the next print time
                print("SE dt={:.3f} ms | e[sL]={:+.4f}  e[sR]={:+.4f}  e[θ]={:+.4f}  e[θ̇]={:+.4f}".format(dt*1e3, float(e[0,0]), float(e[1,0]), float(e[2,0]), float(e[3,0])))
            except Exception:
                pass
            

        yield 0
