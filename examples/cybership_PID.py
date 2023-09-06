 #/usr/bin/env python

from DYSAS import Simulator
from DYSAS.Control import PID
from DYSAS.Navigation import heading_circle
from DYSAS.Utils import deg2rad, kn2ms, rad2deg, ms2kn
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
    
    # Initialize engine load PI controller, tuned for T = 10
    load_contr = PID()
    load_contr.kp = 4.5e-1
    load_contr.ki = 5e-2
    load_contr.usat = [0, 1]
    load_contr.T = T                       # Digital PID model changes with sampling time, must be re-tuned then

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
    sim.vessel.x[2] = deg2rad(0)     # Vessel is headed 0 degrees (clockwise)
    sim.vessel.x[3] = kn2ms(0)       # Vessel is stopped

    # Speed and heading setpoints
    load_contr.r = kn2ms(1)                            # Top speed for cybership model is approximately 8.33 kn
    heading_contr.r = heading_circle(deg2rad(45))      # 45 degrees

    # Store data and plot it later
    y = sim.vessel.x
    u = np.array([0,0])

    # Set up environmental forces (wind)
    sim.vessel.wind.speed = kn2ms(0)
    sim.vessel.wind.dir = deg2rad(0)

    for _ in range(0, sim_time, sim.T):
        # Update engine load controller
        power_perc = load_contr.update(sim.vessel.x[3])
        sim.vessel.power.u = power_perc
        # Update heading controller
        rudder_angle = heading_contr.update(sim.vessel.x[2])
        sim.vessel.steering.u = rudder_angle

        # Simulate for T seconds with a step of dt
        sim.simulate(sim.vessel.get_state_vector())
        u = np.c_[ u, np.array([power_perc, rudder_angle])]

        # Append time
        t.append(t[-1]+T)

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
        plt.plot(t, ms2kn(load_contr.r)*np.ones(len(t)), 'k--', lw=2.0)
        plt.plot(t, [ms2kn(a) for a in sim.vessel.y[3,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Surge speed (kn)')
        
        # Plot heading from simulation
        plt.figure()
        plt.grid()
        plt.plot(t, rad2deg(heading_contr.r)*np.ones(len(t)), 'k--', lw=2.0)
        plt.plot(t, [rad2deg(a) for a in sim.vessel.y[2,:]], lw=2.0)
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (deg)')
        
        # Plot ship position
        plt.figure()
        plt.grid()
        plt.plot(sim.vessel.y[1,:], sim.vessel.y[0,:], lw=2.0)
        plt.title("Ship position")
        plt.xlabel("E (m)")
        plt.ylabel("N (m)")
        # plt.axis([-np.min(sim.vessel.y[1,:])-100, np.max(sim.vessel.y[1,:])+100, -np.min(sim.vessel.y[0,:])-100, np.max(sim.vessel.y[0,:])+100])
        
        # Show figures
        plt.show()
    except ImportError:
        pass

    print(sim.vessel.y[0,-1], sim.vessel.y[1,-1], sim.vessel.y[3,-1])