from DYSAS.Actuator import Propeller, Rudder, GeneralizedForces, Azimuth
from DYSAS.Control import PID
from DYSAS.Engine import DCMotor
from DYSAS.Environment import Wind
from DYSAS.Power import Battery
from DYSAS.Utils import *
from DYSAS.Vessel import Ship

from scipy.integrate import ode
import numpy as np

class Simulator:
    def __init__(self, vessel, dt=1, T=1, solver='dopri5', method='rtol'):
        # The movement model for the vessel
        self.vessel = vessel
    
        # Integration time
        self.dt = dt

        # Sampling time
        self.T = T

        # ODE solver
        self.solver = ode(self.vessel.update).set_integrator(solver, method=method)

        # Simulation time
        self.t = 0

    def simulate(self, init):
        self.solver.set_initial_value(init)        # Set initial values for simulation

        # Simulate based on model for T seconds
        # Number of iterations is equal to T/dt
        while self.solver.successful() and self.solver.t < self.T:
            self.solver.integrate(self.solver.t+self.dt)         # Solve ODE (simulate based on model)
        
        # Store last system states in output vector
        self.vessel.y = np.c_[self.vessel.y, self.vessel.x[:]]

        # Update simulation time
        self.t += self.solver.t

        # Update state
        return self.vessel.x