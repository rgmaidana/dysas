#!/usr/bin/env python

from DYSAS import Simulator
from DYSAS.Vessel import GretheASVPID
from DYSAS.Utils import deg2rad, kn2ms, ms2kn, rad2deg
import numpy as np

if __name__ == "__main__":
    # Simulation parameters
    t = [0]             # Time vector
    dt, T = 1, 10       # Derivation and sampling time
    sim_time = 500      # Simulation time

    # Simulator class
    sim = Simulator(GretheASVPID(), dt=dt, T=T)

    # Controller
    sim.vessel.controller.kp = 5e1
    sim.vessel.controller.ki = 3e2
    sim.vessel.controller.T = 1e-1
    sim.vessel.controller.r = kn2ms(3)
    
    # Initial conditions
    sim.vessel.x[2] = deg2rad(0)
    sim.vessel.x[3] = kn2ms(3)
    sim.vessel.propulsion.u = sim.vessel.controller.update(0)
    
    # Array to store data and plot it later
    y = sim.vessel.x
    u = np.array([sim.vessel.propulsion.u])
    
    # Set up environmental forces (wind)
    sim.vessel.wind.speed = kn2ms(0)
    sim.vessel.wind.dir = deg2rad(0)

    for _ in range(0, sim_time, sim.T):
        # Simulate for T seconds with a step of dt
        sim.simulate(sim.vessel.get_state_vector())     # Simulate ship
        y = np.c_[y, sim.vessel.x[:]]                   # Store last states
        u = np.c_[ u, np.array([sim.vessel.propulsion.u])]

        # Append time
        t.append(t[-1]+T)

    # Plot results
    try:
        import matplotlib.pyplot as plt

        legend = []

        # Plot inputs from simulation
        plt.figure()
        plt.grid()
        for k in range(0,u.shape[0]):
            plt.plot(t, u[k,:], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Inputs')
        legend = []
        for k in range(0,u.shape[0]):
            legend.append('u%d' % (k+1))
        plt.legend(legend)

        # Plot output from simulation
        plt.figure()
        plt.grid()
        for k in range(y.shape[0]):
            plt.plot(t, y[k,:], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('States')
        for k in range(0,y.shape[0]):
            legend.append('y%d' % (k+1))
        plt.legend(legend)
        
        # Plot ship position
        plt.figure()
        plt.grid()
        plt.plot(y[1,:], y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")

        # Plot surge speed from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, [ms2kn(a) for a in y[3,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Surge speed (kn)')

        # Show figures
        plt.show()
    except ImportError:
        pass