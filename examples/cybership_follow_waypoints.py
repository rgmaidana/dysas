#/usr/bin/env python

from DYSAS import Simulator
from DYSAS.Control import PID
from DYSAS.Navigation import Waypoints, RefSystem
from DYSAS.Utils import deg2rad, kn2ms, rad2deg, ms2kn
from DYSAS.Vessel import CyberShip

import numpy as np
from scipy.integrate import ode
import timeit

if __name__ == "__main__":    
    # Simulation parameters
    t = [0]             # Time vector
    dt, T = 1, 10       # Derivation and sampling time
    sim_time = 500      # Simulation time

    # Simulator class
    sim = Simulator(CyberShip(), dt=dt, T=T)

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
    sim.vessel.engine.I = 0
    sim.vessel.propulsion.omega = 0
    sim.vessel.steering.angle = deg2rad(0)
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
    y = np.array([0 for _ in range(len(sim.vessel.x))])    # Output vector with initial states
    y = y.reshape((1, len(sim.vessel.x))).T
    u = np.array([0,0])

    # Run while there are waypoints to follow
    t1 = timeit.default_timer()
    while not refsys.waypoints.empty():
        # Update engine load controller
        power_perc = load_contr.update(sim.vessel.x[3])
        # Update heading controller
        heading_contr.r = refsys.los_guidance(sim.vessel.x[0], sim.vessel.x[1])
        rudder_angle = heading_contr.update(sim.vessel.x[2])
        
        sim.vessel.power.u = power_perc
        sim.vessel.steering.u = rudder_angle
        sim.simulate()
        y = np.c_[ y, np.array(sim.vessel.x[:]).reshape((len(sim.vessel.x),1)) ]    # Store last vessel states
        u = np.c_[ u, np.array([power_perc, rudder_angle])]

        # Append time
        t.append(t[-1]+T)

        # Get next waypoint if reached current within a margin of error (10 meters)
        dist = np.sqrt( (sim.vessel.x[0]-refsys.next[0])**2 + (sim.vessel.x[1]-refsys.next[1])**2 )
        if dist <= 1e1:
            refsys.current = refsys.next
            refsys.next = refsys.waypoints.get()
    t2 = timeit.default_timer() - t1

    print(t2)

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