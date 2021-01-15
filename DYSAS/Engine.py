# coding=utf-8

# TODO: Implement Machinery System Operating mode (MSO-mode) from
# https://github.com/BorgeRokseth/ship_in_transit_simulator

from MarineSystemSim.Utils import saturate
from scipy.integrate import ode

# DC Motor following the common model (e.g., https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling)
class DCMotor:
    def __init__(self, K=0.01, R=1, L=0.5, max_current=1, efficiency=1):
        # Constructive parameters
        self.K = K                         # Torque and back EMF constant
        self.R = R                         # Armature resistance
        self.L = L                         # Armature inductance
        self.efficiency = efficiency       # Motor efficiency (i.e., yield)
        self.max_current = max_current
        
        # Motor state-space (just current, angular speed comes from propeller)
        self.x = [0]                       # Armature current

        # Input (voltage)
        self.u = 0
        
    def diff(self, omega):
        # State derivative
        return [(self.u - self.R*self.x[0] - self.K*omega)/self.L]           # Armature current

    # Torque output
    def output(self):
        I = saturate(self.x[0], -self.max_current, self.max_current)
        return (self.K*I)*self.efficiency

    # def update(self, x, u=0, omega=0):
    #     self.I = (u - self.R*x - self.K*omega)/self.L
    #     return self.I

    # # Perform simulation for some time, and ODE solver internally updates states
    # # Parameters:
    # # dt: ODE derivation time
    # # T:  System sampling time
    # def simulate(self, omega, dt=1, T=1):
    #     self.ode_solver.set_initial_value(self.I)      # Current initial values are the last states
    #     self.ode_solver.set_f_params(self.u, omega)    # System input and angular speed (parameter)

    #     # Simulate based on model for T seconds
    #     # Number of iterations is equal to T/dt
    #     while self.ode_solver.successful() and self.ode_solver.t < T:
    #         self.ode_solver.integrate(self.ode_solver.t+dt)         # Solve ODE (simulate based on model)
        
    #     # Update state
    #     self.I = saturate(self.ode_solver.y, -self.max_current, self.max_current)

    #     # Output torque
    #     return (self.K*self.I)*self.efficiency
    