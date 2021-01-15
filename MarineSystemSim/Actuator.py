# coding=utf-8

from MarineSystemSim.Utils import saturate
from math import copysign, pi
from scipy.integrate import ode

# Class to implement the generalized forces actuator model
class GeneralizedForces:
    def __init__(self):
        # Controller input vector
        self.u = [0 for _ in range(3)]
        
    def output(self):
        return self.u

# Class for implementing the propeller shaft and blades
# Input: Torque from a motor/engine
class Propeller:
    def __init__(self, Is=1, Kq=0, Kt=0, D=0, max_shaft_speed=1, rho=1000):
        # Constructive parameters
        self.Is = Is    # Moment of inertia
        self.Kq = Kq    # Load torque coefficient
        self.Kt = Kt    # Load thrust coefficient
        self.D = D      # Propeller diameter
        self.max_shaft_speed = max_shaft_speed

        # Other parameters
        self.rho = rho  # Water density
        
        # Propeller state-space
        self.x = [0]                # Angular shaft speed

        # Input (torque)
        self.u = 0

    def diff(self):
        # State derivative
        return [(1/self.Is)*( self.u - copysign(1, self.x[0])*self.Kq*self.rho*self.D**5*(self.x[0]/(2*pi))**2 )]

    def output(self):
        # Output thrust
        omega = saturate(self.x[0], -self.max_shaft_speed, self.max_shaft_speed)
        return copysign(1, omega) * self.Kt * self.rho * self.D**4 * (omega/(2*pi))**2

    # def update(self, t, x, u=0):
    #     # Angular shaft speed differential function
    #     # N.B 1! We assume there is no slack or slip in the shaft-propeller junction.
    #     # Therefore, the angular shaft speed should be equal to the angular propeller speed.
    #     # N.B 2! The original differential uses the load torque (Qa) as a function of the propeller
    #     # angular speed in revolutions-per-second. This needs to be converted to rad/s: 1 rev = 2*pi rad
    #     # N.B 3! Last term of model, Kw*omega, is not included here because it is accounted for in the motor model.
    #     # This means the friction is between the rotor and shaft, which agrees with our assumption that
    #     # there is no slack or slip in the shaft-propeller junction.
    #     return (1/self.Is)*( u - copysign(1, x)*self.Kq*self.rho*self.D**5*(x/(2*pi))**2 )

    # # Perform simulation for some time, and ODE solver internally updates states
    # # Parameters:
    # # dt: ODE derivation time
    # # T:  System sampling time
    # def simulate(self, dt=1, T=1):
    #     self.ode_solver.set_initial_value(self.omega)      # Current initial values are the last states
    #     self.ode_solver.set_f_params(self.u)                         # System input

    #     # Simulate based on model for T seconds
    #     # Number of iterations is equal to T/dt
    #     while self.ode_solver.successful() and self.ode_solver.t < T:
    #         self.ode_solver.integrate(self.ode_solver.t+dt)         # Solve ODE (simulate based on model)
        
    #     # Update state
    #     self.omega = self.ode_solver.y

    #     # Output thrust
    #     return copysign(1, self.omega) * self.Kt * self.rho * self.D**4 * (self.omega/(2*pi))**2

# Class for implementing the rudder actuator model
class Rudder:
    def __init__(self, v=0, r=0, max_angle=0):
        # Constructive parameters
        self.v = v
        self.r = r
        self.max_angle = 0

        # Rudder state-space
        self.x = [0]        # Rudder angle

        # Input (commanded rudder angle)
        self.u = 0
        
    # The rudder angle remains constant
    def diff(self):
        # State derivative
        return [0]
    
    def output(self, surge_v):
        # Rudder-to-forces model
        # N.B! Forces in sway and yaw generated by rudder depend on surge speed.
        # Thus, this ship model is underactuated, and there is a strong coupling between motor torque (i.e., propeller thrust) and heading.
        angle = saturate(self.u, -self.max_angle, self.max_angle)
        return -self.v * surge_v * angle, -self.r * surge_v * angle