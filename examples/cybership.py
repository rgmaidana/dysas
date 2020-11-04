#!/usr/bin/env python

from MarineSystemSim.Actuator import ThrusterRudder
from MarineSystemSim.Vessels import Ship
import numpy as np
from scipy.integrate import ode

if __name__ == "__main__":
    # Create vessel instance (CyberShip Drilling Vessel scaled model)
    # Ship states are: East position, North position, Yaw, Velocity east, Velocity north, Velocity yaw
    cs = Ship()

    # Set CyberShip parameters (From Astrid Brodtkorb's matlab parameter files)
    cs.m = 127.92
    cs.Xg = 0
    cs.Iz = 61.967
    
    cs.Xu = -2.332
    cs.Yv = -4.673
    cs.Nr = -0.01675

    cs.Xu_dot = 3.262
    cs.Yv_dot = 28.89; cs.Yr_dot = 0.525
    cs.Nv_dot = 0.157; cs.Nr_dot = 13.98

    # Thruster model parameters from Friedrich's master thesis, using Kt and Kq from Thruster 1: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2415123
    actuator = ThrusterRudder()
    actuator.Kt = 0.3763      # Propeller thrust constant
    actuator.Kq = 0.0113      # Motor torque constant
    actuator.D = 0.03

    # Rudder model parameters
    actuator.c_rudder_v = 0.166
    actuator.c_rudder_r = 1.661

    # Actuator input
    actuator.u[0] = 0.001                         # 1 miliNewton per meter
    actuator.u[1] = 0                             # Straight rudder

    # Simulation
    t = [0]                                 # Time vector
    y = cs.x.reshape((cs.x.shape[0],1))     # Output vector with initial states
    dt, T = 1, 10                           # Derivation and sampling time
    sim_time = 200                          # Simulation time

    # Initial conditions
    cs.x[2] = np.pi/4       # 45 degrees (clockwise)
    cs.x[3] = 2/1.943844    # 2 knots in surge direction

    while True:
        # Update actuator output        
        cs.u = actuator.act(cs.x)

        # Simulate
        cs.simulate(dt=dt, T=T)
        y = np.c_[y, cs.x[:]]             # Store last states

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
        for k in range(y.shape[0]):
            plt.plot(t, y[k,:], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('States')
        legend = []
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
        plt.axis([-100, 100, -100, 100])
        
        # Show figures
        plt.show()
    except ImportError:
        pass