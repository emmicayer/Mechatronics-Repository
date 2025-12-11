## @file state_estimation.py
#  @brief State-estimation helper functions for Romi, which provides 
#  kinematic relationships and utility calculations used for
#  validating or interpreting estimator behavior.
#
#  This driver focuses on simple analytical relationships such as:
#    • Converting wheel speeds to linear and angular velocity
#    • Computing expected robot motion given left/right wheel velocities
#    • Supporting quick checks or reference calculations for estimator outputs

from numpy import array, arange, zeros, transpose, cos, sin, arange, asarray
import numpy as np
from matplotlib import pyplot, rc
from math import pi

# Electromechanical properties
K      = 250*2*pi/60/4.5     # Motor Gain [rad/(V*s)]
tau    = 0.1                 # Motor Time Constant [s]
Kp = 3
r     = 0.035   # Wheel radius [m]
b     = 0.141   # Track width [m]
Vmax  = 4.5     # V
v_ref = 0.15  # tangential speed [m/s]
R_ref = 0.60  # desired circle radius [m]  

def sat(u, umax):
    return np.clip(u, -umax, umax)

# user-provided inputs
def vL_in(t): return 2.0   # volts
def vR_in(t): return 2.0   # volts

# Open loop system model
def system_eqn_OL(t, x):
    u = array([ [ 4.5 ],                                # Open loop inputs
                [ 3.0 ] ])      
    
    x_rate = r*(x[0, 0]+x[1, 0])/2*cos(x[4, 0])         # State equations
    y_rate = r*(x[0, 0]+x[1, 0])/2*sin(x[4, 0])
    phi_rate = (x[0, 0]-x[1, 0])*r/b
    xd_m =  array( [ [ K/tau*u[0,0] - 1/tau*x[0,0] ],
                   [ K/tau*u[1,0] - 1/tau*x[1,0] ], 
                   [x_rate], 
                   [y_rate], 
                   [phi_rate] ])
    
    y_m  =  array( [ [ x[0,0] ],                        # Output equations
                   [ x[1,0] ],
                    [ x[2,0] ], [ x[3,0] ], [ x[4,0] ]])

    return xd_m, y_m

x = array([ [0],      # wL
              [0],      # wR
              [0],      # x pos
              [0],      # y pos
              [0] ])

t = 0
xd_m, y_m = system_eqn_OL(t, x)

# Closed loop  system model
def system_eqn_CL(t, x):
    if x.shape[0] == 2:
        x = np.vstack([x, [[0.0],[0.0],[0.0]]])

    wL = x[0, 0] 
    wR = x[1, 0] 
    x_pos = x[2, 0] 
    y_pos = x[3, 0] 
    th = x[4, 0] 

    omega_ref = v_ref / R_ref                   # Circle                 
    wR_ref = (v_ref + 0.5*b*omega_ref)/r
    wL_ref = (v_ref - 0.5*b*omega_ref)/r

    vL_cmd = np.clip(Kp*(wL_ref - wL), -Vmax, Vmax)         # Wheel-speed P control -> voltages (clip to supply)
    vR_cmd = np.clip(Kp*(wR_ref - wR), -Vmax, Vmax)

    wL_dot = (K*vL_cmd - wL)/tau        # Motor first-order speed dynamics using ONLY K and tau
    wR_dot = (K*vR_cmd - wR)/tau

    v     = r*(wR + wL)/2.0         # Differential-drive kinematics (pose MUST be part of the state)
    omega = r*(wR - wL)/b
    x_dot   = v*cos(th)
    y_dot   = v*sin(th)
    th_dot  = omega

    x_rate = r*(x[0, 0]+x[1, 0])/2*cos(x[4, 0])
    y_rate = r*(x[0, 0]+x[1, 0])/2*sin(x[4, 0])
    phi_rate = (x[0, 0]-x[1, 0])*r/b
    dxdt = array([[wL_dot],[wR_dot],[x_rate], 
                   [y_rate], 
                   [phi_rate] ])
    yout = array([[wL],[wR],[x_pos],[y_pos],[th]])  # Log pose + wheel speeds
    return dxdt, yout

x_0 = array([ [0],
              [0] ])
t_OL, y_OL = RK4_solver(system_eqn_OL, x_0, [0, 1], 0.01)
rc('font', **{'size'   : 16})
pyplot.figure(figsize=(8,6))
pyplot.plot(t_OL, y_OL[:,0])
pyplot.plot(t_OL, y_OL[:,1])
pyplot.legend(["$\Omega_L$","$\Omega_R$"])
pyplot.xlabel('Time, t [s]')
pyplot.ylabel('Motor Velocity [rad/s]')
pyplot.grid()
pyplot.tight_layout()

# Closed loop RK4 
t_CL, y_CL = RK4_solver(system_eqn_CL, x_0, [0, 1], 0.01)

pyplot.figure(figsize=(8,6))
pyplot.plot(t_CL, y_CL[:,0])
pyplot.plot(t_CL, y_CL[:,1])
pyplot.legend(["$\Omega_L$","$\Omega_R$"])
pyplot.xlabel('Time, t [s]')
pyplot.ylabel('Motor Velocity [rad/s]')
pyplot.grid()
pyplot.tight_layout()

# Open loop Romi path
x_0 = array([ [0],      # wL
              [0],      # wR
              [0],      # x pos
              [0],      # y pos
              [0] ])
t_0L, y_0L = RK4_solver(system_eqn_OL, x_0, [0, 5], 0.01)
pyplot.figure(figsize=(8,8))
pyplot.plot(y_0L[:,2], y_0L[:,3])
pyplot.xlabel('X Position, [m]')
pyplot.ylabel('Y Position, [m]')
pyplot.grid()
pyplot.tight_layout()

# Closed loop Romi path
x_0 = array([ [0],      # wL
              [0],      # wR
              [0],      # x pos
              [0],      # y pos
              [0] ])
t_CL, y_CL = RK4_solver(system_eqn_CL, x_0, [0, 5], 0.01)
pyplot.figure(figsize=(8,8))
pyplot.plot(y_0L[:,2], y_0L[:,3])
pyplot.xlabel('X Position, [m]')
pyplot.ylabel('Y Position, [m]')
pyplot.grid()
pyplot.tight_layout()
