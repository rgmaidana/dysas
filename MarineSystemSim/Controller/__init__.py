# coding=utf-8

from numpy import inf

# Digital PID controller class
class PID:
    def __init__(self, kp=0, ki=0, kd=0, usat=[-inf,inf], T=1, aw=True):
        # Controller gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Integrator term
        self.I = 0
        
        # Sampling time
        self.T = T

        # Anti-windup
        self.u = 0
        self.aw = aw

        # Controller output saturation
        self.usat = usat

        # Error vector
        self.e = [0, 0]

        # Setpoint
        self.r = 0
    
    def update(self, y):
        # Setpoint error
        self.e[-1] = self.r - y

        # Proportional
        P = self.kp*self.e[-1]

        # Integral with anti-windup
        if self.aw:
            if (self.usat[0] < self.u and self.u < self.usat[1]) or \
               (self.u > self.usat[1] and self.e[-1] < 0) or \
               (self.u < self.usat[0] and self.e[-1] > 0):
               self.I += self.e[-1]*self.T
        I = self.ki*self.I

        # Derivative
        D = ((self.e[-1] - self.e[-2]) / self.T)*self.kd
        
        # PID controller output with saturation
        self.u = self.saturate(P+I+D)

        # Roll back error vector
        self.e[-2] = self.e[-1]

        return self.u
    
    # Apply saturation to controller output
    def saturate(self, u):
        if u < self.usat[0]:
            u = self.usat[0]
        elif u > self.usat[1]:
            u = self.usat[1]
        return u