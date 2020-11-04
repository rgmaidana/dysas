# Marine Systems Simulation python package

This package implements state-space differential equation models and for simulating marine systems (e.g., ships) in Python 2.7 (master branch) and 3.6 (python36 branch).
Currently this package implements a generic ship state-space model, from the combined kinematic and newtonian dynamic 3 DoF models in the [Marine Cybernetics lecture notes](http://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf).


The user may choose between two input models for the ship: 
* A generalized surge, sway and forces vector (inputs are Surge force, Sway force and Yaw force) .
* A thruster/rudder model (inputs are commanded motor torque and rudder angle).


Package structure based on Thor Fossen and Asgeir Johansen's Marine System Simulator (MSS) MATLAB toolbox.

## Dependencies

* [numpy](https://www.numpy.org/)
* [scipy](https://www.scipy.org/)

## Installation

Install the package directly from PyPI:

```pip install MarineSystemSim```

Or clone the repository and install locally:

```pip -e <path_to_repository>```

## Usage

Simply import the class of controller marine system wanted and instantiate it.
Optional arguments for the constructor function are the differential equation solver and solving method, and which input model to use (e.g., "generalized" or "rudder").

```
from MarineSystemSim.Vessels import Ship

ship = Ship()
```

The ship's constructive parameters may be updated by changing the variables directly:

```
ship.m = 1      # Mass, kg
ship.Xg = 0     # X coordinate for center of gravity, meters
```

Users can find all the parameters in the [Ship class](https://github.com/rgmaidana/python-mss/blob/master/MarineSystemSim/Vessels/__init__.py), and an explanation of the parameters in the [Marine Cybernetics lecture notes](http://folk.ntnu.no/assor/Public/2018-08-20%20marcyb.pdf).

After instantiating and parametrizing the ship, use the `simulate()` function to run the simulation and update the ship states.
The simulate function arguments are:
* dt: ODE solver derivation time
* T:  System sampling time

The simulation runs every "dt" seconds, for "T" seconds. Thus, the number of times the states will be updated is equal to T/dt.
For example, if dt is 0.5 seconds and T is 10 seconds, this means the ship states will be computed every half second for 10 seconds.
This means the ODE solver will run for 10/0.5 = 20 iterations, and thus users may change the simulation coarseness by changing the ratio between T and dt.

The package also includes Controller (e.g., PID) and Actuator (e.g., Thruster-Rudder, Generalized Forces) classes.
To use, for example, the PID controller:

```
from MarineSystemSim.Controller import PID

controller = PID()
controller.kp = 1
controller.ki = 0
controller.kd = 0
controller.T = 1
```

To use, for example, the Thruster-Rudder model:

```
from MarineSystemSim.Actuator import ThrusterRudder

actuator = ThrusterRudder()
actuator.Kt = 1
actuator.Kq = 1
actuator.D = 1
actuator.c_rudder_r = 1
actuator.c_rudder_v = 1
```

The [Cybership PID](https://github.com/rgmaidana/python-mss/blob/master/examples/cybership_PID.py) example uses the PID controller and Thruster-Rudder actuator model.

## Examples

This package currently contains 3 [examples](https://github.com/rgmaidana/predictiveControl/tree/master/examples):

* Model and simulation of NTNU's Gunnerus Research Vessel, using the generalized forces input model;
* Model and simulation of NTNU's CyberShip Drilling Vessel scaled model, using the thruster/rudder model;
* Model, simulation and linear PID control (i.e., surge speed and heading control) of NTNU's CyberShip Drilling Vessel scaled model, using the thruster/rudder model.

You can run the examples with:

```python <example>.py```

If you want to plot the results, you must install the matplotlib package:

```pip install matplotlib```

## Acknowledgements

* [Thor I. Fossen](https://www.ntnu.edu/employees/thor.fossen)
* [Asgeir J. Sørensen](https://www.ntnu.edu/employees/asgeir.sorensen)
* [MATLAB Marine Systems Simulator toolbox](https://github.com/cybergalactic/MSS)

## Collaborators

* [Renan Maidana](https://github.com/rgmaidana)
