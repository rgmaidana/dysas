#/usr/bin/env python

from MarineSystemSim import Simulator
from MarineSystemSim.Actuator import Propeller, Rudder
from MarineSystemSim.Controller import PID
from MarineSystemSim.Engine import DCMotor
from MarineSystemSim.Navigation import Waypoints, RefSystem
from MarineSystemSim.Power import Battery
from MarineSystemSim.Utils import deg2rad, kn2ms, rad2deg, ms2kn
from MarineSystemSim.Vessel import Ship

import numpy as np
from scipy.integrate import ode
import timeit

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
    # Note: A 12VDC motor can only very slightly move a 90kg ship model. Thus, we comment the motor's armature resistance and inductance
    # and leave in the default values (0.01 ohms and 0.5 H), which will generate a higher EMF to move the ship further.
    motor = DCMotor()
    motor.K = 0.3           # Linear friction coefficient
    # Leave these commented for now so we can see the vessel move more than half a centimeter
    # motor.R = 72e-3       # From OS OMA-2820-950 datasheet, called phase resistance (?)
    # motor.L = 24e-6       # From OS OMA-2820-950 datasheet, called phase inductance (?)
    motor.efficiency = 0.8  # From OS OMA-2890-950 datasheet
    motor.max_current = 22  # From OS OMA-2890-950 datasheet

    # Propeller/shaft model parameters from Friedrich's master thesis, using Kt and Kq from Thruster 1: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2415123
    # Note 2: Here the same logic as with the DC motor applies: A 3 cm propeller can only slightly move forward the heavy vessel model.
    # We increase the propeller diameter by 10 times so the ship can move further.
    propeller = Propeller()
    propeller.Kt = 0.3763           # Load thrust coefficient
    propeller.Kq = 0.0113           # Load torque coefficient
    propeller.D = 0.3               # Propeller diameter (increased from model to move the ship forward)
    propeller.Is = 25000.0/747225.0 # Moment of inertia
    propeller.max_shaft_speed = 157 

    # Rudder model parameters
    rudder = Rudder()
    rudder.v = 0.166
    rudder.r = 1.661
    rudder.max_angle = deg2rad(30)  # Rudder can only turn +- 30 degrees
    
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

    # Initialize engine load PI controller, tuned for T = 10
    load_contr = PID()
    load_contr.kp = 4.5e-1
    load_contr.ki = 5e-2
    load_contr.usat = [0, 1]
    load_contr.T = T                       # Digital PID model changes with sampling time, must be re-tuned then
    load_contr.r = kn2ms(1)                # Top speed for cybership model is approximately 8.33 kn

    # Initialize heading PD controller, tuned for T = 10
    heading_contr = PID()
    heading_contr.kp = 1e-1
    heading_contr.kd = 3.5e0
    heading_contr.usat = [deg2rad(-30), deg2rad(30)]   # Rudder can move +- 30 degrees
    heading_contr.T = T

    # Initial conditions
    sim.engine.I = 0                
    sim.propulsion.omega = 0
    sim.steering.angle = deg2rad(0)
    sim.vessel.x[2] = deg2rad(0)    # Vessel is headed 0 degrees (clockwise)
    sim.vessel.x[3] = kn2ms(0)      # Vessel is stopped

    # Waypoints for the ship to follow (queue of [x,y] coordinates)
    # Zig-zag towards north direction
    wp_list = [[200,200], [300,100], [400,200], [500,100], [500,100]]
    wps = Waypoints()
    wps.fromlist(wp_list)

    # Reference system
    refsys = RefSystem(r=50, waypoints=wps)
    refsys.current = sim.vessel.x[0:2]  # First waypoint is initial location of vessel
    refsys.next = refsys.waypoints.get()
    
    # Store data and plot it later
    y = np.array([0 for _ in range(len(cybership.x))])    # Output vector with initial states
    y = y.reshape((1, len(cybership.x))).T
    u = np.array([0,0])

    # Run while there are waypoints to follow
    t1 = timeit.default_timer()
    while not refsys.waypoints.empty():
        # Update engine load controller
        power_perc = load_contr.update(cybership.x[3])
        # Update heading controller
        heading_contr.r = refsys.los_guidance(sim.vessel.x[0], sim.vessel.x[1])
        rudder_angle = heading_contr.update(cybership.x[2])
        
        sim.energy.u = power_perc
        sim.steering.u = rudder_angle
        sim.simulate()
        y = np.c_[ y, np.array(cybership.x[:]).reshape((len(cybership.x),1)) ]    # Store last vessel states
        u = np.c_[ u, np.array([power_perc, rudder_angle])]

        # Append time
        t.append(t[-1]+T)

        # Get next waypoint if reached current within a margin of error (10 meters)
        dist = np.sqrt( (sim.vessel.x[0]-refsys.next[0])**2 + (sim.vessel.x[1]-refsys.next[1])**2 )
        if dist <= 1e1:
            refsys.current = refsys.next
            refsys.next = refsys.waypoints.get()
    t2 = timeit.default_timer() - t1

    print t2

    # Plot results
    try:
        import matplotlib.pyplot as plt
        
        # Plot ship position
        plt.figure()
        plt.grid()
        plt.plot(y[1,:], y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")
        # plt.axis([-np.min(y[1,:])-100, np.max(y[1,:])+100, -np.min(y[0,:])-100, np.max(y[0,:])+100])
        
        # Plot waypoints
        for wp in wp_list:
            plt.plot(wp[1], wp[0], '-ko', markersize=12)

        # Show figures
        plt.show()
    except ImportError:
        pass

    print(y[0,-1], y[1,-1], y[3,-1])