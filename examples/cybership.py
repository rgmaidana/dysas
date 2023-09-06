#!/usr/bin/env python

from DYSAS import Simulator
from DYSAS.Utils import deg2rad, kn2ms, ms2kn
from DYSAS.Vessel import CyberShip

import numpy as np
from scipy.integrate import ode

if __name__ == "__main__":
    # Simulation parameters
    t = [0]             # Time vector
    dt, T = 1, 10       # Derivation and sampling time
    sim_time = 500      # Simulation time

    # Simulator class
    sim = Simulator(CyberShip(), dt=dt, T=T)

    # Initial conditions
    sim.vessel.engine.I = 0
    sim.vessel.propulsion.omega = 0
    sim.vessel.steering.angle = deg2rad(0)
    sim.vessel.x[2] = deg2rad(30)   # Vessel is headed 30 degrees (clockwise)
    sim.vessel.x[3] = kn2ms(0)      # Vessel is stopped

    sim.vessel.power.u = 1

    # Array to store data and plot it later
    y = sim.vessel.x

    # Set up environmental forces (wind)
    sim.vessel.wind.speed = kn2ms(0)
    sim.vessel.wind.dir = deg2rad(0)

    for _ in range(0, sim_time, sim.T):       
        # Set desired speed as state (assumes perfect control)
        # sim.vessel.x[3] = kn2ms(1)
        
        # Simulate for T seconds with a step of dt
        sim.simulate(sim.vessel.get_state_vector())

        # Append time
        t.append(t[-1]+T)
        if t[-1] >= sim_time:     # If end of simulation, break loop
            break

    # Plot results
    try:
        import matplotlib.pyplot as plt

        # Plot output from simulation
        plt.figure()
        plt.grid()
        for k in range(sim.vessel.y.shape[0]):
            plt.plot(t, sim.vessel.y[k,:], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('States')
        legend = []
        for k in range(0,sim.vessel.y.shape[0]):
            legend.append('y%d' % (k+1))
        plt.legend(legend)
        
        # Plot ship position
        plt.figure()
        plt.grid()
        plt.plot(sim.vessel.y[1,:], sim.vessel.y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")
        # plt.axis([-100, 100, -100, 100])

        # Plot surge speed from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, [ms2kn(a) for a in sim.vessel.y[3,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Surge speed (kn)')

        # Show figures
        plt.show()
    except ImportError:
        pass