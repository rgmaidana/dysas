# coding=utf-8

from math import pi
from numpy.linalg import norm

### Set of utility functions common to all modules

def kn2ms(x):
    return 1852*x/3600.0

def ms2kn(x):
    return 3600*x/1852.0

def nmi2m(x):
    return 1852*x

def m2nmi(x):
    return x/1852.0

def deg2rad(x):
    return x*pi/180

def rad2deg(x):
    return x*180/pi

def saturate(x, a, b):
    return max(a, min(x, b))