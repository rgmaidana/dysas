# coding=utf-8

from math import pi
import pymap3d as pm    # Use version 2.9.1 until they fix the Ellipsoid class documentation

rho = 1025  # Density of salt water

# Class to replace a component (e.g., power, engine) when there is none
class NullComponent:
    def __init__(self):
        self.u = [0]
        self.x = [0]

    def diff(self):
        return [0]
    
    def output(self):
        return 0

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

def lla2enu(lat, lon, lat0, lon0, h=0, h0=0, proj='wgs84'):
    return pm.geodetic2enu(lat, lon, h, lat0, lon0, h0, ell=pm.Ellipsoid(proj))

def lla2ecef(lat, lon, proj='wgs84', h=0):
    return pm.geodetic2ecef(lat, lon, h, ell=pm.Ellipsoid(proj))

def lla2ned(lat, lon, lat0, lon0, h=0, h0=0, proj='wgs84'):
    return pm.geodetic2ned(lat, lon, h, lat0, lon0, h0, ell=pm.Ellipsoid(proj))

def enu2lla(e, n, lat0, lon0, u=0, h0=0, proj='wgs84'):
    return pm.enu2geodetic(e, n, u, lat0, lon0, h0, ell=pm.Ellipsoid(proj))

def ecef2lla(x, y, z=0, proj='wgs84'):
    return pm.ecef2geodetic(x, y, z, ell=pm.Ellipsoid(proj))

def ned2lla(n, e, lat0, lon0, d=0, h0=0, proj='wgs84'):
    return pm.ned2geodetic(n, e, d, lat0, lon0, h0, ell=pm.Ellipsoid(proj))