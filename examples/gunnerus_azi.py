#!/usr/bin/env python

from turtle import heading
from DYSAS import Simulator
from DYSAS.Control import PID
from DYSAS.Navigation import heading_circle
from DYSAS.Vessel import GunnerusAzi
from DYSAS.Utils import deg2rad, rad2deg, kn2ms, ms2kn
import numpy as np

if __name__ == "__main__":
    # Simulation parameters
    t = [0]               # Time vector
    dt, T = 1, 10         # Derivation and sampling time
    sim_time = 500       # Simulation time

    # Simulator class
    sim = Simulator(GunnerusAzi(), dt=dt, T=T)

    # Initial conditions
    sim.vessel.x[2] = deg2rad(0)    # Vessel is headed 30 degrees (clockwise)
    sim.vessel.x[3] = kn2ms(0)      # Vessel is stopped

    # Array to store data and plot it later
    y = sim.vessel.x
    u = np.array([0,0])
    
    # Set up environmental forces (wind)
    sim.vessel.wind.speed = kn2ms(48.6)     # Approx. 25 m/s
    sim.vessel.wind.dir = deg2rad(45)

    # Desired speed and heading
    sim.vessel.speed_contr.r = kn2ms(13.6)
    sim.vessel.heading_contr.r = heading_circle(deg2rad(45))

    for _ in range(0, sim_time, sim.T):
        # Update speed controller
        rpm = sim.vessel.speed_contr.update(sim.vessel.x[3])
        sim.vessel.propulsion.u[0] = rpm
        # Update heading controller
        angle = sim.vessel.heading_contr.update(sim.vessel.x[2])
        sim.vessel.propulsion.u[1] = angle

        # Simulate for T seconds with a step of dt
        sim.simulate(sim.vessel.get_state_vector())     # Simulate ship
        u = np.c_[ u, np.array([rpm, angle])]

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
        
        # Plot ship position
        plt.figure()
        plt.grid()
        plt.plot(sim.vessel.y[1,:], sim.vessel.y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")

        # Plot surge speed from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, ms2kn(sim.vessel.speed_contr.r)*np.ones(len(t)), 'k--', lw=2.0)
        plt.plot(t, [ms2kn(a) for a in sim.vessel.y[3,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Surge speed (kn)')
        
        # Plot heading from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, rad2deg(sim.vessel.heading_contr.r)*np.ones(len(t)), 'k--', lw=2.0)
        plt.plot(t, [rad2deg(a) for a in sim.vessel.y[2,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (deg)')

        # Show figures
        plt.show()
    except ImportError:
        pass