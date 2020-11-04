#/usr/bin/env python

from MarineSystemSim.Actuator import ThrusterRudder
from MarineSystemSim.Controller import PID
from MarineSystemSim.Vessels import Ship
import numpy as np
from scipy.integrate import ode

# Convert m/s -> kn
def ms2kn(a):
    return a*1.943844

# Convert kn -> m/s
def kn2ms(a):
    return a/1.943844

# Convert deg -> rad
def deg2rad(a):
    return a * np.pi/180

# Convert rad -> deg
def rad2deg(a):
    return a * 180/np.pi

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

    # Thruster model parameters from Friedrich 2016 master thesis, using Kt and Kq from Thruster 1: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2415123
    actuator = ThrusterRudder()
    actuator.Kt = 0.3763      # Propeller thrust constant
    actuator.Kq = 0.0113      # Motor torque constant
    actuator.D = 0.03

    # Rudder model parameters
    actuator.c_rudder_v = 0.166
    actuator.c_rudder_r = 1.661

    # Simulation
    t = [0]                                 # Time vector
    y = cs.x.reshape((cs.x.shape[0],1))     # Output vector with initial states
    u = actuator.u.reshape((actuator.u.shape[0],1))     # Actuator output vector
    dt, T = 1, 10                           # Derivation and sampling time
    sim_time = 200                          # Simulation time

    # Initialize speed PI controller, tuned for T = 10
    speed_contr = PID()
    speed_contr.kp = 8.5e-3
    speed_contr.ki = 1.75e-4
    speed_contr.usat = [-0.5, 0.5]
    speed_contr.T = T                       # Digital PID model changes with sampling time, must be re-tuned then

    # Initialize heading PD controller, tuned for T = 10
    heading_contr = PID()
    heading_contr.kp = 1e-2
    heading_contr.kd = 4.2e-1
    heading_contr.usat = [deg2rad(-30), deg2rad(30)]   # Rudder can move +- 30 degrees
    heading_contr.T = T

    # Initial conditions
    cs.x[2] = deg2rad(0)             # 0 degrees north
    cs.x[3] = kn2ms(0)               # 0 surge speed, ship was previously station-keeping

    # Speed and heading setpoints
    speed_contr.r = kn2ms(7)            # Top speed for cybership model is approximately 8.33 kn
    heading_contr.r = deg2rad(45)       # 45 degrees

    while True:
        # Update speed and heading controllers
        actuator.u[0] = speed_contr.update(cs.x[3])
        actuator.u[1] = heading_contr.update(cs.x[2])
        
        # Update actuator output
        cs.u = actuator.act(cs.x)
        
        # Simulate
        cs.simulate(dt=dt, T=T)
        y = np.c_[y, cs.x[:]]                   # Store last states
        u = np.c_[u, actuator.u[:]]             # Store last inputs

        # Append time
        t.append(t[-1]+T)
        if t[-1] >= sim_time:     # If end of simulation, break loop
            break

    # Plot results
    try:
        import matplotlib.pyplot as plt

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

        # Plot surge speed from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, ms2kn(speed_contr.r)*np.ones(len(t)), 'k--', lw=2.0)
        plt.plot(t, [ms2kn(a) for a in y[3,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Surge speed (kn)')
        
        # Plot heading from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, rad2deg(heading_contr.r)*np.ones(len(t)), 'k--', lw=2.0)
        plt.plot(t, [rad2deg(a) for a in y[2,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (deg)')
        
        # Plot ship position
        plt.figure()
        plt.grid()
        plt.plot(y[1,:], y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")
        # plt.axis([-np.min(y[1,:])-100, np.max(y[1,:])+100, -np.min(y[0,:])-100, np.max(y[0,:])+100])
        
        # Show figures
        plt.show()
    except ImportError:
        pass

    print(y[0,-1], y[1,-1], y[3,-1])