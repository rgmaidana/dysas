# coding=utf-8

import numpy as np
from math import pi, sqrt, sin, cos
from scipy.integrate import ode

# Digital PID controller class
class PID:
    def __init__(self, kp=0, ki=0, kd=0, usat=[-np.inf,np.inf], T=1, aw=True):
        # Controller gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Integrator term
        self.I = 0
        
        # Sampling time
        self.T = T

        # Anti-windup
        self.u = 0
        self.aw = aw

        # Controller output saturation
        self.usat = usat

        # Error vector
        self.e = [0, 0]

        # Setpoint
        self.r = 0
    
    def update(self, y):
        # Setpoint error
        self.e[-1] = self.r - y

        # Proportional
        P = self.kp*self.e[-1]

        # Integral with anti-windup
        if self.aw:
            if (self.usat[0] < self.u and self.u < self.usat[1]) or \
               (self.u > self.usat[1] and self.e[-1] < 0) or \
               (self.u < self.usat[0] and self.e[-1] > 0):
               self.I += self.e[-1]*self.T
        I = self.ki*self.I

        # Derivative
        D = ((self.e[-1] - self.e[-2]) / self.T)*self.kd
        
        # PID controller output with saturation
        self.u = self.saturate(P+I+D)

        # Roll back error vector
        self.e[-2] = self.e[-1]

        return self.u
    
    # Apply saturation to controller output
    def saturate(self, u):
        if u < self.usat[0]:
            u = self.usat[0]
        elif u > self.usat[1]:
            u = self.usat[1]
        return u

class Ship:
    # State-space model of a generic ship with 3 DoF.
    # From Marine Cybernetics lecture notes (Sørensen, 2018): http://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf
    # Combines kinematic model from chapter 6 (i.e., Kinematics) and newtonian model from chapter 7 (i.e., Control-plant model: Vessel model).
    #
    # The 6 states are: N, E, yaw, vx, vy, vyaw (North, East, Yaw, and their velocities).
    # The state transition functions are the states' derivates in time (i.e., their differential equations).
    # Inputs depend if using thruster/rudder model or generalized forces, selected with the "input_model" flag.
    #
    # The ship includes PID control for surge speed and heading.
    # These need to be tuned by the user according to their ship parameters and the system sampling time.

    def __init__(self, solver='dopri5', method='rtol', input_model='generalized'):
        # Ship constructive parameters
        self.m = 1      # Mass, kg
        self.Xg = 0     # X coordinate for center of gravity, m
        self.Iz = 0     # Moment of inertia for Z axis (i.e., heave axis), kg.m^2
            
        # Linear damping parameters
        self.Xu = 0
        self.Yv = 0; self.Yr = 0
        self.Nv = 0; self.Nr = 0
        
        # Inertia matrix parameters
        self.Xu_dot = 0
        self.Yv_dot = 0; self.Yr_dot = 0
        self.Nv_dot = 0; self.Nr_dot = 0

        # Restorative forces parameters
        self.Xx = 0
        self.Yy = 0
        self.Npsi = 0

        # Thruster model
        self.Kt = 0
        self.Kq = 0
        self.D = 0

        # Rudder model
        self.c_rudder_v = 0
        self.c_rudder_r = 0

        # Forward speed control
        self.speed_contr = PID()

        # Heading control
        self.heading_contr = PID()

        # State vector
        self.x = np.zeros(6)

        # Simulation engine
        # Select "rudder" to run thruster/rudder input model, and anything else for generalized forces model
        if input_model == 'rudder':
            print "[Ship] Using thruster/rudder model"
            self.ode_solver = ode(self.update_rudder).set_integrator(solver, method=method)
            self.u = np.zeros(2)
        else:
            print "[Ship] Using generalized forces model"
            self.ode_solver = ode(self.update).set_integrator(solver, method=method)
            self.u = np.zeros(3)

    # Speed and heading control
    def control(self):
        # For speed controller, measured output is last state of surge speed (x4)
        thr = self.speed_contr.update(self.x[3])
        
        # For heading control, measured output is last state of yaw (x3)
        ang = self.heading_contr.update(self.x[2])

        self.u[0], self.u[1] = thr, ang

    # Apply control input to ship model (i.e., state-space transition functions) considering generalized forces
    # Parameters:
    # t = Simulation time (not used in function, is updated by ODE solver)
    # x = System states at t-1
    # u = System inputs (Forces in surge, sway, and yaw directions)
    def update(self, t, x, u=0):             
        # Precalculate sine and cosine for speed
        cs, ss = np.cos(x[2]), np.sin(x[2])
        
        # Generalized forces
        Tx = u[0]       # Surge
        Ty = u[1]       # Sway
        Tpsi = u[2]     # Yaw

        # Helper variables
        a1 = self.Iz*self.Yv_dot - self.Nr_dot*self.Yv_dot \
            + self.Nv_dot*self.Yr_dot - self.Iz*self.m + self.Nr_dot*self.m \
            + (self.Xg*self.m)**2 - self.Nv_dot*self.Xg*self.m - self.Xg*self.Yr_dot*self.m
        a2 = Ty + self.Yr*x[2] * self.Yv*x[1]
        a3 = Tpsi + self.Nr*x[2] * self.Nv*x[1]

        # State transition functions
        f1 = x[3]*cs - x[4]*ss
        f2 = x[3]*ss + x[4]*cs
        f3 = x[5]
        f4 = (Tx + self.Xu*x[3])/(self.m-self.Xu_dot)
        f5 = (1/a1)*( (self.Xg*self.m - self.Yr_dot)*a3 + (self.Nr_dot - self.Iz)*a2 )
        f6 = (1/a1)*( (self.m - self.Yv_dot)*a3 + (self.m*self.Xg - self.Nv_dot)*a2 )

        return np.array([f1, f2, f3, f4, f5, f6]).T

    # Apply control input to ship model (i.e., state-space transition functions) considering thruster/rudder model
    # Parameters:
    # t = Simulation time (not used in function, is updated by ODE solver)
    # x = System states at t-1
    # u = System inputs (Commanded motor torque and rudder angle)
    def update_rudder(self, t, x, u=0):             
        # Precalculate sine and cosine for speed
        cs, ss = np.cos(x[2]), np.sin(x[2])
        
        # Torque-to-thrust model (force in surge generated by propeller thrust)
        Tx = (self.Kt/(self.Kq*self.D))*u[0]

        # Rudder-to-forces model
        # N.B! Forces in sway and yaw generated by rudder depend on surge speed.
        # Thus, this ship model is underactuated, and there is a strong coupling between motor torque (i.e., propeller thrust) and heading.
        Ty = -self.c_rudder_v * u[1] * x[3]
        Tpsi = -self.c_rudder_r * u[1] * x[3]

        # Helper variables
        a1 = self.Iz*self.Yv_dot - self.Nr_dot*self.Yv_dot \
            + self.Nv_dot*self.Yr_dot - self.Iz*self.m + self.Nr_dot*self.m \
            + (self.Xg*self.m)**2 - self.Nv_dot*self.Xg*self.m - self.Xg*self.Yr_dot*self.m
        a2 = Ty + self.Yr*x[2] * self.Yv*x[1]
        a3 = Tpsi + self.Nr*x[2] * self.Nv*x[1]

        # State transition functions
        f1 = x[3]*cs - x[4]*ss
        f2 = x[3]*ss + x[4]*cs
        f3 = x[5]
        f4 = (Tx + self.Xu*x[3])/(self.m-self.Xu_dot)
        f5 = (1/a1)*( (self.Xg*self.m - self.Yr_dot)*a3 + (self.Nr_dot - self.Iz)*a2 )
        f6 = (1/a1)*( (self.m - self.Yv_dot)*a3 + (self.m*self.Xg - self.Nv_dot)*a2 )

        return np.array([f1, f2, f3, f4, f5, f6]).T        

    # Perform simulation for some time, and ODE solver internally updates states
    # Parameters:
    # dt: ODE derivation time
    # T:  System sampling time
    def simulate(self, dt=1, T=1):
        self.ode_solver.set_initial_value(self.x)      # Current initial values are the last states
        self.ode_solver.set_f_params(self.u)           # System input

        # Simulate based on model for T seconds
        # Number of iterations is equal to T/dt
        while self.ode_solver.successful() and self.ode_solver.t < T:
            self.ode_solver.integrate(self.ode_solver.t+dt)     # Solve ODE (simulate based on model)
        
        # Update states
        self.x = self.ode_solver.y