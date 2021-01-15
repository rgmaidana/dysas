# coding=utf-8

from collections import deque
from MarineSystemSim.Utils import deg2rad, rad2deg
from math import pi, sqrt, sin, cos, atan, atan2

# Map a 0->360 degrees angle to 0->180 or 0->(-180) degrees
def heading_circle(x):
    if x > pi:
        return x-2*pi
    elif x < -pi:
        return x+2*pi
    return x

# Helper class to implement a waypoint queue
class Waypoints:
    def __init__(self):
        self.elements = deque()
    
    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()
    
    def fromlist(self, x):
        for i in range(len(x)):
            self.put(x[i])

# Class to implement a reference system (gives the controller/vessel model the desired setpoints)
class RefSystem:
    def __init__(self, r=450, waypoints=Waypoints(), **kwargs):
        self.r = r                      # Lookahead distance
        self.waypoints = waypoints      # Waypoint list
        self.current = []               # Current (or last visited) waypoint (x, y)
        self.next = []                  # Next waypoint (x, y)

    # Line-of-sight guidance (from https://github.com/BorgeRokseth/ship_in_transit_simulator)
    def los_guidance(self, x, y):
        ''' Returns the desired heading (i.e. reference signal to
            a ship heading controller).
        '''
        dx = self.next[0]-self.current[0]
        dy = self.next[1]-self.current[1]

        alpha_k = atan2(dy, dx)
        e_ct = -(x - self.current[0]) * sin(alpha_k) + (y - self.current[1]) * cos(alpha_k)
        
        if e_ct**2 > self.r**2:
            e_ct = 0.99 * self.r
        
        delta = sqrt(self.r**2 - e_ct**2)
        chi_r = atan(-e_ct / delta)
        return alpha_k + chi_r