# coding=utf-8

from numpy import array, zeros

# Class to implement the generalized forces actuator model
class GeneralizedForces:
    def __init__(self):
        # Controller input vector
        self.u = zeros(3)
        
    def act(self):
        return self.u

# Class to implement the thruster/rudder actuator model
class ThrusterRudder:
    def __init__(self, Kt=0, Kq=0, D=0, v=0, r=0):
        # Thruster
        self.Kt = Kt
        self.Kq = Kq
        self.D = D

        # Rudder
        self.c_rudder_v = v
        self.c_rudder_r = r

        # Controller input vector
        self.u = zeros(2)
        
    def act(self, x):
        # Torque-to-thrust model (force in surge generated by propeller thrust)
        Tx = (self.Kt/(self.Kq*self.D))*self.u[0]

        # Rudder-to-forces model
        # N.B! Forces in sway and yaw generated by rudder depend on surge speed.
        # Thus, this ship model is underactuated, and there is a strong coupling between motor torque (i.e., propeller thrust) and heading.
        Ty = -self.c_rudder_v * self.u[1] * x[3]
        Tpsi = -self.c_rudder_r * self.u[1] * x[3]

        return array([Tx, Ty, Tpsi])