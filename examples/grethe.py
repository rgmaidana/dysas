#!/usr/bin/env python

from DYSAS import Simulator
from DYSAS.Vessel import GretheASV
from DYSAS.Utils import deg2rad, kn2ms, ms2kn, rad2deg

if __name__ == "__main__":
    # Simulation parameters
    t = [0]             # Time vector
    dt, T = 1, 10       # Derivation and sampling time
    sim_time = 500      # Simulation time

    # Simulator class
    sim = Simulator(GretheASV(), dt=dt, T=T)

    # Initial conditions
    sim.vessel.x[2] = deg2rad(0)   # Vessel is headed 30 degrees (clockwise)
    sim.vessel.x[3] = kn2ms(0)     # Vessel is stopped

    # Array to store data and plot it later
    sim.vessel.y = sim.vessel.x

    # Surge force to generate approximately 3 kn
    sim.vessel.propulsion.u = 7.12e2/2
    
    # Set up environmental forces (wind)
    sim.vessel.wind.speed = kn2ms(0)
    sim.vessel.wind.dir = deg2rad(0)

    for _ in range(0, sim_time, sim.T):
        # Simulate for T seconds with a step of dt
        sim.simulate(sim.vessel.get_state_vector())     # Simulate ship

        # Append time
        t.append(t[-1]+T)

    # Plot results
    try:
        import matplotlib.pyplot as plt

        legend = []

        # Plot output from simulation
        plt.figure()
        plt.grid()
        for k in range(sim.vessel.y.shape[0]):
            plt.plot(t, sim.vessel.y[k,:], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('States')
        for k in range(0,sim.vessel.y.shape[0]):
            legend.append('y%d' % (k+1))
        plt.legend(legend)
        
        # Plot ship position
        plt.figure(); plt.axis('equal')
        plt.grid()
        plt.plot(sim.vessel.y[1,:], sim.vessel.y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")

        # Plot surge speed from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, [ms2kn(a) for a in sim.vessel.y[3,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Surge speed (kn)')

        # Plot heading from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, [rad2deg(a) for a in sim.vessel.y[2,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (deg)')
        
        # Show figures
        plt.show()
    except ImportError:
        pass