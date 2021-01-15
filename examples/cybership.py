#!/usr/bin/env python

from DYSAS import Simulator
from DYSAS.Actuator import Propeller, Rudder
from DYSAS.Engine import DCMotor
from DYSAS.Power import Battery
from DYSAS.Utils import deg2rad, kn2ms, ms2kn
from DYSAS.Vessel import Ship

import numpy as np
from scipy.integrate import ode

if __name__ == "__main__":
    # Create vessel instance (CyberShip Drilling Vessel scaled model)
    # Ship states are: East position, North position, Yaw, Velocity east, Velocity north, Velocity yaw
    cybership = Ship()

    # Set CyberShip parameters (From Astrid Brodtkorb's matlab parameter files)
    cybership.m = 127.92
    cybership.Xg = 0
    cybership.Iz = 61.967
    
    cybership.Xu = -2.332
    cybership.Yv = -4.673
    cybership.Nr = -0.01675

    cybership.Xu_dot = 3.262
    cybership.Yv_dot = 28.89; cybership.Yr_dot = 0.525
    cybership.Nv_dot = 0.157; cybership.Nr_dot = 13.98

    # Power source
    bat = Battery(voltage=12)

    # Motor/Engine
    motor = DCMotor()
    motor.K = 0.3           # Linear friction coefficient
    # Uncertain parameters, leave commented for now so we can see the vessel move more than half a centimeter
    # motor.R = 72e-3         # From OS OMA-2820-950 datasheet, called phase resistance (?)
    # motor.L = 24e-6         # From OS OMA-2820-950 datasheet, called phase inductance (?)
    motor.efficiency = 0.8  # From OS OMA-2890-950 datasheet
    motor.max_current = 22  # From OS OMA-2890-950 datasheet

    # Propeller/shaft model parameters from Friedrich's master thesis, using Kt and Kq from Thruster 1: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2415123
    propeller = Propeller()
    propeller.Kt = 0.3763           # Load thrust coefficient
    propeller.Kq = 0.0113           # Load torque coefficient
    propeller.D = 0.03              # Propeller diameter
    propeller.Is = 25000.0/747225.0 # Moment of inertia
    propeller.max_shaft_speed = 157 

    # Rudder model parameters
    rudder = Rudder()
    rudder.v = 0.166
    rudder.r = 1.661
    rudder.max_angle = deg2rad(45)  # Rudder can only turn +- 30 degrees

    # Simulation parameters
    t = [0]             # Time vector
    dt, T = 1, 10       # Derivation and sampling time
    sim_time = 500      # Simulation time

    # Simulator class
    sim = Simulator(dt=dt, T=T)
    sim.energy = bat
    sim.engine = motor
    sim.propulsion = propeller
    sim.steering = rudder
    sim.vessel = cybership

    # Initial conditions
    sim.engine.I = 0                
    sim.propulsion.omega = 0
    sim.steering.angle = deg2rad(0)
    sim.vessel.x[2] = deg2rad(30)   # Vessel is headed 30 degrees (clockwise)
    sim.vessel.x[3] = kn2ms(0)      # Vessel is stopped

    # Array to store data and plot it later
    y = sim.vessel.x

    while True:
        sim.energy.u = 1
        sim.simulate()
        y = np.c_[ y, np.array(cybership.x[:]).reshape((len(cybership.x),1)) ]    # Store last vessel states

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
        # plt.axis([-100, 100, -100, 100])

        plt.figure()
        plt.grid()
        plt.plot(y[0,:], [ms2kn(a) for a in y[3,:]])
        plt.xlabel("x1")
        plt.ylabel("x4")

        # Show figures
        plt.show()
    except ImportError:
        pass