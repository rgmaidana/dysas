# coding=utf-8

from math import cos, sin, atan2

class Wind:
    def __init__(self, spd=0, dir=0) -> None:
        self.speed = spd
        self.dir = dir

        # Constants
        self.rho_a = 1.204    # Air density at 20 degrees Celsius
        # See page 191 from Fossen's Handbook of Marine Craft Hyd. and Motion
        # Control.
        self.cx = 0.5
        self.cy = 0.7
        self.cn = 0.08

    # Compute wind forces affecting a vessel in surge, sway and yaw
    def forces(self, vessel):
        # Calculate frontal and lateral projected areas
        # Frontal: Breadth * mean_height_above_water
        AFw = vessel.Br*vessel.Fb
        # Lateral: Length_over_all * mean_height_above_water
        ALw = vessel.LoA*vessel.Fb
        
        # See page 190 from Fossen's Handbook of Marine Craft Hyd. and Motion
        # Control.
        uw = self.speed * cos(self.dir - vessel.x[2])
        vw = self.speed * sin(self.dir - vessel.x[2])
        u_rw = uw - vessel.x[3]
        v_rw = vw - vessel.x[4]
        gamma_rw = -atan2(v_rw, u_rw)
        wind_rw2 = u_rw ** 2 + v_rw ** 2
        c_x = -self.cx * cos(gamma_rw)
        c_y = self.cy * sin(gamma_rw)
        c_n = self.cn * sin(2 * gamma_rw)
        tau_coeff = 0.5 * self.rho_a * wind_rw2
        
        # See page 191 from Fossen's Handbook of Marine Craft Hyd. and Motion
        # Control.        
        tau_u = tau_coeff * c_x * AFw
        tau_v = tau_coeff * c_y * ALw
        tau_n = tau_coeff * c_n * ALw * vessel.LoA

        return [tau_u, tau_v, tau_n]