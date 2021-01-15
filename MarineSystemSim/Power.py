# coding=utf-8

# Electrical battery class
class Battery:
    def __init__(self, voltage=1):
        self.voltage = voltage      # Rated voltage
        self.charge = 0             # Battery's max charge (e.g., 10 000 mAh)

        # Battery state
        self.x = [voltage]          # Battery's current voltage level (diminishes with time and load, not implemented)

        # Input
        self.u = 0
    
    # Charge level and discharging not yet implemented, so voltage level is constant
    def diff(self):
        # State derivative
        return [0]

    # Returns a percentage of the voltage level
    def output(self):
        return self.u * self.x[0]