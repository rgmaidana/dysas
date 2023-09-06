# coding=utf-8

from math import inf
from DYSAS import *
from math import cos, sin
import numpy as np

class Ship:
    # State-space model of a generic ship with 3 DoF.
    # Source: Sørensen, 2018 - http://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf
    # Combines kinematic model from chapter 6 (i.e., Kinematics) and newtonian model from chapter 7 (i.e., Control-plant model: Vessel model).
    #
    # The 6 states are: N, E, yaw, vx, vy, vyaw (North, East, Yaw, and their velocities).
    # Inputs are forces in the surge, sway and yaw directions
    
    def __init__(self): 
        # State vector
        self.x = [0 for _ in range(6)]
        
        # Input vector
        self.u = [0 for _ in range(3)]

        # Output vector (for now stores the simulation history)
        self.y = np.zeros((6,1))

        # Identifying parameters (normally from AIS)
        self.mssi = None    # MSSI number
        self.name = None    # Ship name
        self.clas = None    # Ship class
        self.fs = None      # Flag state

        # Ship constructive parameters
        self.Br = 0     # Breadth, m
        self.LoA = 0    # Length-over-All, m
        self.Fb = 0     # Freeboard (height from waterline), m
        self.Dr = 0     # Draught
        self.m = 0      # Mass, kg
        self.Xg = 0     # X coordinate for center of gravity, m
        self.Iz = 0     # Moment of inertia for Z axis (i.e., heave axis), kg.m^2
            
        # Inertia matrix elements
        self.m11 = 0
        self.m22 = 0; self.m23 = 0
        self.m32 = 0; self.m33 = 0

        # Linear damping matrix elements
        self.d11 = 0
        self.d22 = 0; self.d23 = 0
        self.d32 = 0; self.d33 = 0

        # Restorative forces matrix elements
        self.g11 = 0
        self.g22 = 0
        self.g33 = 0

        # Linear damping parameters
        # self.Xu = 0
        # self.Yv = 0; self.Yr = 0
        # self.Nv = 0; self.Nr = 0
        
        # # Inertia matrix parameters
        # self.Xud = 0
        # self.Yvd = 0; self.Yrd = 0
        # self.Nvd = 0; self.Nrd = 0

        # # Restorative forces parameters
        # self.Xx = 0
        # self.Yy = 0
        # self.Npsi = 0

        # Environmental disturbances
        self.wind = Wind()

        self.constant_speed = False
    
    # Update state-space
    def update(self, x):
        self.x = x

    def diff(self):             
        # Precalculate sine and cosine for speed
        cs, ss = cos(self.x[2]), sin(self.x[2])
        
        # Helper variables
        a1 = self.Iz*self.Yv_dot - self.Nr_dot*self.Yv_dot \
            + self.Nv_dot*self.Yr_dot - self.Iz*self.m + self.Nr_dot*self.m \
            + (self.Xg*self.m)**2 - self.Nv_dot*self.Xg*self.m - self.Xg*self.Yr_dot*self.m
        a2 = self.u[1] + self.Yr*self.x[2] * self.Yv*self.x[1]
        a3 = self.u[2] + self.Nr*self.x[2] * self.Nv*self.x[1]

        # State transition functions
        f1 = self.x[3]*cs - self.x[4]*ss
        f2 = self.x[3]*ss + self.x[4]*cs
        f3 = self.x[5]
        f4 = (self.u[0] + self.Xu*self.x[3])/(self.m-self.Xu_dot)
        f5 = (1/a1)*( (self.Xg*self.m - self.Yr_dot)*a3 + (self.Nr_dot - self.Iz)*a2 )
        f6 = (1/a1)*( (self.m - self.Yv_dot)*a3 + (self.m*self.Xg - self.Nv_dot)*a2 )

        # States derivative
        return [f1, f2, f3, f4, f5, f6]

    # # Ship movement model (i.e., state-space transition functions)
    # # Parameters:
    # # t = Simulation time (not used in function, is updated by ODE solver)
    # # x = System states at t-1
    # # u = System inputs (Commanded forces in surge, sway, and yaw directions)
    # def update(self, t, x, u=0):             
    #     # Precalculate sine and cosine for speed
    #     cs, ss = cos(x[2]), sin(x[2])
        
    #     # Helper variables
    #     a1 = self.Iz*self.Yv_dot - self.Nr_dot*self.Yv_dot \
    #         + self.Nv_dot*self.Yr_dot - self.Iz*self.m + self.Nr_dot*self.m \
    #         + (self.Xg*self.m)**2 - self.Nv_dot*self.Xg*self.m - self.Xg*self.Yr_dot*self.m
    #     a2 = u[1] + self.Yr*x[2] * self.Yv*x[1]
    #     a3 = u[2] + self.Nr*x[2] * self.Nv*x[1]

    #     # State transition functions
    #     f1 = x[3]*cs - x[4]*ss
    #     f2 = x[3]*ss + x[4]*cs
    #     f3 = x[5]
    #     f4 = (u[0] + self.Xu*x[3])/(self.m-self.Xu_dot)
    #     f5 = (1/a1)*( (self.Xg*self.m - self.Yr_dot)*a3 + (self.Nr_dot - self.Iz)*a2 )
    #     f6 = (1/a1)*( (self.m - self.Yv_dot)*a3 + (self.m*self.Xg - self.Nv_dot)*a2 )

    #     return [f1, f2, f3, f4, f5, f6]

    # # Perform simulation for some time, and ODE solver internally updates states
    # # Parameters:
    # # dt: ODE derivation time
    # # T:  System sampling time
    # def simulate(self, dt=1, T=1):
    #     self.ode_solver.set_initial_value(self.x)      # Current initial values are the last states
    #     self.ode_solver.set_f_params(self.u)           # System input

    #     # Simulate based on model for T seconds
    #     # Number of iterations is equal to T/dt
    #     while self.ode_solver.successful() and self.ode_solver.t < T:
    #         self.ode_solver.integrate(self.ode_solver.t+dt)     # Solve ODE (simulate based on model)
        
    #     # Update states
    #     self.x = self.ode_solver.y