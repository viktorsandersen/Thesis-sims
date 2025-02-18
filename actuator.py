import numpy as np


class ActuatorMotor:
    """
    Class to update mujoco actuator parameters, to change between position, velocity and torque control
    code with example of usage: https://github.com/lvjonok/mujoco-actuators-types/blob/master/demo.ipynb
    """
    def __init__(self) -> None:
        self.dyn = np.array([1, 0, 0])
        self.gain = np.array([1, 0, 0])
        self.bias = np.array([0, 0, 0])
        self.ctrlrange = np.array([-100, 100])

    def __repr__(self) -> str:
        return f"ActuatorMotor(dyn={self.dyn}, gain={self.gain}, bias={self.bias})"


class ActuatorPosition(ActuatorMotor):
    def __init__(self, kp=200, kd=100) -> None:
        super().__init__()
        self.kp = kp
        self.kd = kd
        self.gain[0] = self.kp
        self.bias[1] = -self.kp
        self.bias[2] = -self.kd
        self.ctrlrange = np.array([-2*np.pi, 2*np.pi])


class ActuatorVelocity(ActuatorMotor):
    def __init__(self, kv=5000) -> None:
        super().__init__()
        self.kv = kv
        self.gain[0] = self.kv
        self.bias[2] = -self.kv
        self.ctrlrange = np.array([-2*np.pi, 2*np.pi])


def update_actuator(model, actuator_id, actuator, ctrlrange=None):
    """
    Update actuator in model

    Parameters
    ----------
    model - mujoco.MjModel
    actuator_id - int or str (name) (for reference see, named access to model elements)
    actuator - ActuatorMotor, ActuatorPosition, ActuatorVelocity\
    ctrlrange - np.array, optional, control range for actuator
    """

    model.actuator(actuator_id).dynprm[:3] = actuator.dyn
    model.actuator(actuator_id).gainprm[:3] = actuator.gain
    model.actuator(actuator_id).biasprm[:3] = actuator.bias
    actuator.ctrlrange = ctrlrange if ctrlrange is not None else actuator.ctrlrange
    model.actuator(actuator_id).ctrlrange[:2] = actuator.ctrlrange


