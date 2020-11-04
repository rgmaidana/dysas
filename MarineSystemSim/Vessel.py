# coding=utf-8

from numpy import zeros, array, cos, sin
from scipy.integrate import ode

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
        
        # State vector
        self.x = zeros(6)
        
        # Input vector
        self.u = zeros(3)

        # Simulation engine
        self.ode_solver = ode(self.update).set_integrator(solver, method=method)
        
    # Ship movement model (i.e., state-space transition functions)
    # Parameters:
    # t = Simulation time (not used in function, is updated by ODE solver)
    # x = System states at t-1
    # u = System inputs (Commanded forces in surge, sway, and yaw directions)
    def update(self, t, x, u=0):             
        # Precalculate sine and cosine for speed
        cs, ss = cos(x[2]), sin(x[2])
        
        # Helper variables
        a1 = self.Iz*self.Yv_dot - self.Nr_dot*self.Yv_dot \
            + self.Nv_dot*self.Yr_dot - self.Iz*self.m + self.Nr_dot*self.m \
            + (self.Xg*self.m)**2 - self.Nv_dot*self.Xg*self.m - self.Xg*self.Yr_dot*self.m
        a2 = u[1] + self.Yr*x[2] * self.Yv*x[1]
        a3 = u[2] + self.Nr*x[2] * self.Nv*x[1]

        # State transition functions
        f1 = x[3]*cs - x[4]*ss
        f2 = x[3]*ss + x[4]*cs
        f3 = x[5]
        f4 = (u[0] + self.Xu*x[3])/(self.m-self.Xu_dot)
        f5 = (1/a1)*( (self.Xg*self.m - self.Yr_dot)*a3 + (self.Nr_dot - self.Iz)*a2 )
        f6 = (1/a1)*( (self.m - self.Yv_dot)*a3 + (self.m*self.Xg - self.Nv_dot)*a2 )

        return array([f1, f2, f3, f4, f5, f6]).T 

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