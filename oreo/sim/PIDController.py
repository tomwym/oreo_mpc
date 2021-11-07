import numpy as np

class ActuatorPIDConroller(object):

    def __init__(self, nameString, origin):
        # 30, 50, 8
        self.Kp = 45
        self.Ki = 180
        self.Kd = 8

        self.current_e = 0
        self.previous_e = 0
        self.sum_e = 0

        self.name = nameString

        self.error = 0
        self.Derivator = 0
        self.Integrator = 0

        self.Integrator_max = 20
        self.Integrator_min = 0

        self.origin = np.asarray(origin)


    def getPID(self, setpoint, position):
        # https://code.activestate.com/recipes/577231-discrete-pid-controller/
        # origin is size 3 tuple of the actuator position at phi = psi = 0
        # target is a size 2 tuple of magnitude distances from actuator origin
        # current_post is a size 3 tuple of coordinates of an actuator
        
        self.error = setpoint - position
        self.P_value = self.Kp * self.error

        dedt = ( self.error - self.Derivator ) * 260
        dedt_limit = 10
        dedt = saturation(dedt, -dedt_limit, dedt_limit)

        self.D_value = self.Kd * dedt
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error
        self.Integrator = saturation(self.Integrator, self.Integrator_min, self.Integrator_max)
        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value + 0.0325


        force_limit = 20
        PID = saturation(PID, -force_limit, force_limit)

        return PID


def saturation(value, min, max):
    if value > max:
        value = max
    if value < min:
        value = min
    return value
