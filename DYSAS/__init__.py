from DYSAS.Actuator import Propeller, Rudder, GeneralizedForces
from DYSAS.Controller import PID
from DYSAS.Engine import DCMotor
from DYSAS.Power import Battery
from DYSAS import Utils
from DYSAS.Vessel import Ship

from scipy.integrate import ode

class Simulator:
    def __init__(self, dt=1, T=1, solver='dopri5', method='rtol'):
        # ODE solver
        self.solver = ode(self.update).set_integrator(solver, method=method)

        # Energy source: Energy source to the engine
        self.energy = Battery()

        # Vessel engine: Generates torque from power
        self.engine = DCMotor()

        # Propulsion: Generates thrust (forward movement) from the engine torque
        self.propulsion = Propeller()    

        # Steering: Steers the ship
        self.steering = Rudder()

        # Vessel: The movement model for the ship
        self.vessel = Ship()
    
        # Integration time
        self.dt = dt

        # Sampling time
        self.T = T

    def update(self, t, x):
        # States derivative vector
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.energy.x)
        self.energy.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.engine.x)
        self.engine.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.propulsion.x)
        self.steering.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.vessel.x)
        self.vessel.x = x[st:ed].tolist()
        
        # Power
        dx += self.energy.diff()                        # Get the power source states' derivatives
        power = self.energy.output()                    # Get the power (input to engine)

        # Engine
        self.engine.u = power
        dx += self.engine.diff(self.propulsion.x[0])    # Get engine states' derivatives
        torque = self.engine.output()                   # Get engine torque (input to propulsion)

        # Propulsion
        self.propulsion.u = torque
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Steering
        dx += self.steering.diff()                                      # Get the steering states' derivatives
        f_sway, f_yaw = self.steering.output(self.vessel.x[3])          # Get the forces in sway and yaw (inputs to movement model)

        # Vessel
        self.vessel.u[0], self.vessel.u[1], self.vessel.u[2] = thrust, f_sway, f_yaw
        dx += self.vessel.diff()                  
        
        return dx

    def simulate(self):
        # The number of states in each component is variable, but the "state_vector" order must be maintained.
        # Else, the order on the update function above must be changed as well.
        # Furthermore, each "diff" function must return the same number of arguments as the number of states.
        state_vector = self.energy.x + self.engine.x + self.propulsion.x + self.steering.x + self.vessel.x
        self.solver.set_initial_value(state_vector)        # Current initial values are the last states

        # Simulate based on model for T seconds
        # Number of iterations is equal to T/dt
        while self.solver.successful() and self.solver.t < self.T:
            self.solver.integrate(self.solver.t+self.dt)         # Solve ODE (simulate based on model)
        
        # Update state
        return self.vessel.x