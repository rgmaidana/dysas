# coding=utf-8

from math import inf
from DYSAS import *
from math import cos, sin
import numpy as np

class Ship:
    # State-space model of a generic ship with 3 DoF.
    # Source: Sørensen, 2018 - http://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf
    # Combines kinematic model from chapter 6 (i.e., Kinematics) and newtonian model from chapter 7 (i.e., Control-plant model: Vessel model).
    #
    # The 6 states are: N, E, yaw, vx, vy, vyaw (North, East, Yaw, and their velocities).
    # Inputs are forces in the surge, sway and yaw directions
    
    def __init__(self): 
        # State vector
        self.x = [0 for _ in range(6)]
        
        # Input vector
        self.u = [0 for _ in range(3)]

        # Output vector (for now stores the simulation history)
        self.y = np.zeros((6,1))

        # Identifying parameters (normally from AIS)
        self.mssi = None    # MSSI number
        self.name = None    # Ship name
        self.clas = None    # Ship class
        self.fs = None      # Flag state

        # Ship constructive parameters
        self.Br = 0     # Breadth, m
        self.LoA = 0    # Length-over-All, m
        self.Fb = 0     # Freeboard (height from waterline), m
        self.Dr = 0     # Draught
        self.m = 0      # Mass, kg
        self.Xg = 0     # X coordinate for center of gravity, m
        self.Iz = 0     # Moment of inertia for Z axis (i.e., heave axis), kg.m^2
            
        # Inertia matrix elements
        self.m11 = 0
        self.m22 = 0; self.m23 = 0
        self.m32 = 0; self.m33 = 0

        # Linear damping matrix elements
        self.d11 = 0
        self.d22 = 0; self.d23 = 0
        self.d32 = 0; self.d33 = 0

        # Restorative forces matrix elements
        self.g11 = 0
        self.g22 = 0
        self.g33 = 0

        # Linear damping parameters
        # self.Xu = 0
        # self.Yv = 0; self.Yr = 0
        # self.Nv = 0; self.Nr = 0
        
        # # Inertia matrix parameters
        # self.Xud = 0
        # self.Yvd = 0; self.Yrd = 0
        # self.Nvd = 0; self.Nrd = 0

        # # Restorative forces parameters
        # self.Xx = 0
        # self.Yy = 0
        # self.Npsi = 0

        # Environmental disturbances
        self.wind = Wind()

        self.constant_speed = False
    
    # Update state-space
    def update(self, x):
        self.x = x

    def diff(self):             
        # Precalculate sine and cosine for efficiency
        cs, ss = cos(self.x[2]), sin(self.x[2])
        
        # Environmental disturbances on surge, sway and yaw velocities
        tau_env = self.wind.forces(self)

        # Helper variables
        a1 = self.m22*self.m33 - self.m23*self.m32
        a2 = tau_env[1] + self.u[1] + self.d22*self.x[4] + self.d23*self.x[5] + cs*self.g22*self.x[1] - self.g11*ss*self.x[0]
        a3 = tau_env[2] + self.u[2] + self.d32*self.x[4] + self.d33*self.x[5] + self.g33*self.x[2]

        # State transition functions (i.e., state derivatives)
        f1 = self.x[3]*cs - self.x[4]*ss
        f2 = self.x[3]*ss + self.x[4]*cs
        f3 = self.x[5]
        f4 = (tau_env[0] + self.u[0] + self.d11*self.x[3] + cs*self.g11*self.x[0] + self.g22*ss*self.x[1])/self.m11
        f5 = (1/a1)*(self.m33*a2 - self.m23*a3)
        f6 = (1/a1)*(self.m22*a3 - self.m32*a2)
        
        if not self.constant_speed:
            return [f1, f2, f3, f4, f5, f6]
        return [f1, f2, f3, 0, f5, f6]

## Some examples of ships
class Gunnerus(Ship):
    def __init__(self):
        super().__init__()
        # Constructive parameters
        self.Br = 9.9
        self.Fb = 4.2 - 2.7 # Depth - draught
        self.Dr = 2.7
        self.LoA = 31.25
        self.m = 418061
        self.Xg = 13.202
        self.Iz = self.m*7.225**2  # From moment of inertia formula: mass x (radius of gyration on z axis)^2
        
        self.Xu = 17400
        self.Yv = 80200
        self.Nr = 4971500

        self.Xud = 70000
        self.Yvd = 340000; self.Yrd = 5749241.32
        self.Nvd = self.Yrd; self.Nrd = 20240000.51

        self.m11 = self.m - self.Xud
        self.m22 = self.m - self.Yvd; self.m23 = self.m*self.Xg - self.Yrd
        self.m32 = self.m*self.Xg - self.Nvd; self.m33 = self.Iz - self.Nrd

        self.d11 = -self.Xu
        self.d22 = -self.Yv
        self.d33 = -self.Nr

        self.propulsion = GeneralizedForces()

    def get_state_vector(self):
        return self.propulsion.x + self.x

    def update(self, t, x):
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()
    
        # Propulsion
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Vessel
        self.u[0], self.u[1], self.u[2] = thrust, 0, 0
        dx += self.diff()
        
        return dx

# Gunnerus with Azimuth thrusters
# See https://github.com/NTNU-MCS/MCSim_python/tree/main/models/RVG_maneuvering for more details
class GunnerusAzi(Ship):
    def __init__(self):
        super().__init__()
        # Constructive parameters
        self.Br = 9.9
        self.Fb = 4.2 - 2.7 # Depth - draught
        self.Dr = 2.7
        self.LoA = 31.25
        self.m = 418061
        self.Xg = 13.202
        self.Iz = self.m*7.225**2  # From moment of inertia formula: mass x (radius of gyration on z axis)^2
        
        self.Xu = 17400
        self.Yv = 80200
        self.Nr = 4971500

        self.Xud = 70000
        self.Yvd = 340000; self.Yrd = 5749241.32
        self.Nvd = self.Yrd; self.Nrd = 20240000.51

        self.m11 = self.m - self.Xud
        self.m22 = self.m - self.Yvd; self.m23 = self.m*self.Xg - self.Yrd
        self.m32 = self.m*self.Xg - self.Nvd; self.m33 = self.Iz - self.Nrd

        self.d11 = -self.Xu
        self.d22 = -self.Yv
        self.d33 = -self.Nr

        # Gunnerus has 2 azimuth thrusters - here we consider an equivalent thruster
        # at the ship midline
        self.propulsion = Azimuth()
        self.propulsion.Ct = 1.8        # Thrust force coefficient
        self.propulsion.Ap = 9          # Foil projected area
        self.propulsion.Al = 1          # Lift parameter
        self.propulsion.Ad = 0.4        # Drag parameter
        self.propulsion.Cd0 = 0.2       # Base drag
        # Thruster position (x,y,z) in body frame
        self.propulsion.Pt = [-15.5600097, 0, -3.9]

        # Surge speed PI controller, tuned for T = 10
        self.speed_contr = PID()
        self.speed_contr.kp = 20
        self.speed_contr.ki = 3
        self.speed_contr.usat = [0, inf]
        self.speed_contr.T = 10

        # Heading PD controller, tuned for T = 10
        self.heading_contr = PID()
        self.heading_contr.kp = 2.5e-2
        self.heading_contr.ki = 0
        self.heading_contr.kd = 1e-3
        self.heading_contr.usat = [-self.propulsion.maxAngle, self.propulsion.maxAngle]   # Azimuth thruster can turn +- 30 degrees
        self.heading_contr.T = 10

    def get_state_vector(self):
        return self.propulsion.x + self.x

    def update(self, t, x):
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()
    
        # Propulsion
        dx += self.propulsion.diff()                                # Get propulsion states' derivatives
        Fx, Fy, Nz = self.propulsion.output(self.x[3], self.x[4])   # Get rigid-body forces from azimuth thruster

        # Vessel
        self.u[0], self.u[1], self.u[2] = Fx, Fy, Nz
        dx += self.diff()
        
        return dx

class CyberShip(Ship):
    def __init__(self):
        super().__init__()
        
        # Constructive parameters
        self.Br = 0.44
        self.Fb = 0.211 - 0.133 # Depth - draught
        self.LoA = 2.578
        self.m = 127.92
        self.Xg = 0
        self.Iz = 61.967
        
        self.Xu = -2.332
        self.Yv = -4.673  
        self.Nr = -0.01675

        self.Xud = 3.262
        self.Yvd = 28.89; self.Yrd = 0.525
        self.Nvd = 0.157; self.Nrd = 13.98

        # Power source
        self.power = Battery(voltage=12)

        # Motor/Engine
        # Note: A 12VDC motor can only very slightly move a 90kg ship model. Thus, we comment the motor's armature resistance and inductance
        # and leave in the default values (0.01 ohms and 0.5 H), which will generate a higher EMF to move the ship further.
        self.engine = DCMotor()
        self.engine.K = 0.3           # Linear friction coefficient
        # Leave these commented for now so we can see the vessel move more than half a centimeter
        # self.engine.R = 72e-3       # From OS OMA-2820-950 datasheet, called phase resistance (?)
        # self.engine.L = 24e-6       # From OS OMA-2820-950 datasheet, called phase inductance (?)
        self.engine.efficiency = 0.8  # From OS OMA-2890-950 datasheet
        self.engine.max_current = 22  # From OS OMA-2890-950 datasheet

        # Propeller/shaft model parameters from Friedrich's master thesis, using Kt and Kq from Thruster 1: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2415123
        # Note 2: Here the same logic as with the DC motor applies: A 3 cm propeller can only slightly move forward the heavy vessel model.
        # We increase the propeller diameter by 10 times so the ship can move further.
        self.propulsion = Propeller()
        self.propulsion.Kt = 0.3763           # Load thrust coefficient
        self.propulsion.Kq = 0.0113           # Load torque coefficient
        self.propulsion.D = 0.3               # Propeller diameter (increased from model to move the ship forward)
        self.propulsion.Is = 25000.0/747225.0 # Moment of inertia
        self.propulsion.max_shaft_speed = 157 

        # Rudder model parameters
        self.steering = Rudder()
        self.steering.v = 0.166
        self.steering.r = 1.661
        self.steering.max_angle = deg2rad(30)  # Rudder can only turn +- 30 degrees

    def get_state_vector(self):
        return self.power.x + self.engine.x + self.propulsion.x + self.steering.x + self.x

    def update(self, t, x):
        # States derivative vector
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.power.x)
        self.power.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.engine.x)
        self.engine.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.propulsion.x)
        self.steering.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()
        
        # Power
        dx += self.power.diff()                        # Get the power source states' derivatives
        power = self.power.output()                    # Get the power (input to engine)

        # Engine
        self.engine.u = power
        dx += self.engine.diff(self.propulsion.x[0])    # Get engine states' derivatives
        torque = self.engine.output()                          # Get engine torque (input to propulsion)

        # Propulsion
        self.propulsion.u = torque
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Steering
        dx += self.steering.diff()                                      # Get the steering states' derivatives
        f_sway, f_yaw = self.steering.output(self.x[3])          # Get the forces in sway and yaw (inputs to movement model)

        # Ship movement model
        self.u[0], self.u[1], self.u[2] = thrust, f_sway, f_yaw
        dx += self.diff()     
        
        return dx

class GretheASV(Ship):
    def __init__(self):
        super().__init__()
        
        # Constructive parameters
        self.Br = 2.15
        self.Fb = 1.308-0.3 # From assembly drawings: Hull height from 462 mm - draught without motor
        self.Dr = 0.7       # Draught with motor
        self.LoA = 5.374
        self.m = 810
        
        self.m11 = self.m*1.15
        self.m22 = self.m*2
        self.m33 = self.m22*2**2

        self.d11 = -self.Br*0.3*0.7*rho
        self.d22 = -self.LoA*0.3*0.7*rho
        self.d33 = -0.5*(self.LoA**2*0.3*0.7*rho)
        
        self.propulsion = GeneralizedForces()
        
        # Would be nice to model the engine and propeller, but
        # couldn't find much data on them...
        # Stuff below is copied from Cybership model
        # # Power source
        # self.power = Battery(voltage=24)

        # # Torqueedo Cruise 2.0 engine
        # self.engine = DCMotor()
        # self.engine.K = 0.3           # Linear friction coefficient
        # # Leave these commented for now so we can see the vessel move more than half a centimeter
        # # self.engine.R = 72e-3       # From OS OMA-2820-950 datasheet, called phase resistance (?)
        # # self.engine.L = 24e-6       # From OS OMA-2820-950 datasheet, called phase inductance (?)
        # self.engine.efficiency = 0.8  # From OS OMA-2890-950 datasheet
        # self.engine.max_current = 22  # From OS OMA-2890-950 datasheet

        # # Propeller/shaft model parameters from Friedrich's master thesis, using Kt and Kq from Thruster 1: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2415123
        # self.propulsion = Propeller()
        # self.propulsion.Kt = 0.3763           # Load thrust coefficient
        # self.propulsion.Kq = 0.0113           # Load torque coefficient
        # self.propulsion.D = 0.3048            # Propeller diameter (12 inches)
        # self.propulsion.Is = 25000.0/747225.0 # Moment of inertia
        # self.propulsion.max_shaft_speed = 157 

        # # Rudder model parameters
        # self.steering = Rudder()
        # self.steering.v = 0.166
        # self.steering.r = 1.661
        # self.steering.max_angle = deg2rad(30)  # Rudder can only turn +- 30 degrees

    def get_state_vector(self):
        return self.propulsion.x + self.x

    def update(self, t, x):
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()
    
        # Propulsion
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Vessel
        self.u[0], self.u[1], self.u[2] = thrust, 0, 0
        dx += self.diff()
        
        return dx
        
class GunnerusPID(Gunnerus):
    def __init__(self):
        super().__init__()
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.controller = PID(self.kp, self.ki, self.kd)
    
    def update(self, t, x):
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()
    
        # Control
        torque = self.controller.update(self.x[3])
        self.propulsion.u = torque

        # Propulsion
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Vessel
        self.u[0], self.u[1], self.u[2] = thrust, 0, 0
        dx += self.diff()
        
        return dx
        
class GretheASVPID(GretheASV):
    def __init__(self):
        super().__init__()
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.controller = PID(self.kp, self.ki, self.kd)
    
    def update(self, t, x):
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()
    
        # Control
        torque = self.controller.update(self.x[3])
        self.propulsion.u = torque

        # Propulsion
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Vessel
        self.u[0], self.u[1], self.u[2] = thrust, 0, 0
        dx += self.diff()
        
        return dx
    
class GretheASVRudder(GretheASV):
    def __init__(self):
        super().__init__()
        self.load_ctrl    = PID(1, 0, 0, aw=True)
        self.heading_ctrl = PID(1, 0 ,0)
        self.max_power = 1120 # Watts
        self.load_ctrl.usat = [-self.max_power, self.max_power]

        self.propulsion = Propeller()
        self.propulsion.Kt = 0.2867           # Load thrust coefficient
        self.propulsion.D = 0.32              # Propeller diameter (12.6 inches)
        # Load torque coefficient
        self.propulsion.Kq = self.propulsion.Kt*self.propulsion.D
        self.propulsion.Is = 25000.0/747225.0 # Moment of inertia
        self.propulsion.max_shaft_speed = 157

        # Rudder model parameters
        self.steering = Rudder()
        self.steering.v = 0.166
        self.steering.r = 1.661
        self.steering.max_angle = deg2rad(61)  # Rudder can only turn +- 61 degrees

    def get_state_vector(self):
        return self.propulsion.x + self.steering.x + self.x
    
    def update(self, t, x):
        # States derivative vector
        dx = []

        # Update modules' internal states
        st, ed = 0, len(self.propulsion.x)
        self.propulsion.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.propulsion.x)
        self.steering.x = x[st:ed].tolist()
        st, ed = ed, ed+len(self.x)
        self.x = x[st:ed].tolist()

        # Propulsion
        dx += self.propulsion.diff()                    # Get propulsion states' derivatives
        thrust = self.propulsion.output()               # Get thrust (input to movement model)

        # Steering
        dx += self.steering.diff()                                      # Get the steering states' derivatives
        f_sway, f_yaw = self.steering.output(self.x[3])          # Get the forces in sway and yaw (inputs to movement model)

        # Ship movement model
        self.u[0], self.u[1], self.u[2] = thrust, f_sway, f_yaw
        dx += self.diff()     
        
        return dx