#!/usr/bin/env python

from MSS.Vessels import Ship
import numpy as np
from scipy.integrate import ode

if __name__ == "__main__":
    # Create vessel instance (NTNU's Gunnerus Research Vessel)
    # Ship states are: East position, North position, Yaw, Velocity east, Velocity north, Velocity yaw
    gunnerus = Ship()

    # Set gunnerus' constructive parameters (From Marine Cybernetics lecture notes, Appendix D: http://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf)
    gunnerus.m = 418061
    gunnerus.Xg = 13.202
    gunnerus.Iz = gunnerus.m*7.225**2  # From moment of inertia formula: mass x (radius of gyration on z axis)^2
    
    gunnerus.Xu = 17400
    gunnerus.Yv = 80200
    gunnerus.Nr = 4971500

    gunnerus.Xu_dot = -70000
    gunnerus.Yv_dot = -340000; gunnerus.Yr_dot = 5749241.32
    gunnerus.Nv_dot = gunnerus.Yr_dot; gunnerus.Nr_dot = -20240000.51

    # Initial conditions (All zero in this case)
    gunnerus.x = np.zeros(gunnerus.x.shape[0])

    # Input
    gunnerus.u[0] = 1                                      # 1 N/s on surge direction

    # Simulation
    t = [0]                                                # Time vector
    y = gunnerus.x.reshape((gunnerus.x.shape[0],1))        # Output vector with initial states
    T = 10                                                 # Sampling time
    sim_time = 200                                         # Simulation time
    
    while True:
        gunnerus.simulate(dt=1, T=10)           # Simulate ship
        y = np.c_[y, gunnerus.x[:]]             # Store last states

        # Append time
        t.append(t[-1]+T)
        if t[-1] >= sim_time:     # If end of simulation, break loop
            break

    # Plot results
    try:
        import matplotlib.pyplot as plt

        legend = []

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
        
        # Show figures
        plt.show()
    except ImportError:
        pass