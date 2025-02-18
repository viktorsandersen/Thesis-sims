# bounding box for position of tcp
class BoundingBox():
    def __init__(self, xlim, ylim, zlim):
        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim

    def check_position(self, position):
        if (position[0] > self.xlim[0] and position[0] < self.xlim[1] and
                position[1] > self.ylim[0] and position[1] < self.ylim[1] and
                position[2] > self.zlim[0] and position[2] < self.zlim[1]):
            return True
        else:
            print("Position out of bounds")
            return False


class VelocityBounding():
    def __init__(self, max_velocity):
        self.max_velocity = max_velocity
    
    def check_velocity(self, velocity):
        # turn x y z velocity into magnitude
        total_vel = (velocity[0]**2 + velocity[1]**2 + velocity[2]**2)**0.5
        if (total_vel < self.max_velocity):
            return True
        else:
            print("Velocity out of bounds")
            return False
        
class AccelerationBounding():
    def __init__(self, max_acceleration, max_ang_acceleration):
        self.max_acceleration = max_acceleration
        self.max_ang_acceleration = max_ang_acceleration

    def check_acceleration(self, acceleration):
        # turn x y z acceleration into magnitude
        total_acc = (acceleration[0]**2 + acceleration[1]**2 + acceleration[2]**2)**0.5
        total_ang_acc = (acceleration[3]**2 + acceleration[4]**2 + acceleration[5]**2)**0.5
        if (total_acc < self.max_acceleration and total_ang_acc < self.max_ang_acceleration):
            return True
        else:
            print(f"Acceleration out of bounds {total_acc} {total_ang_acc}")
            return False

class ForceBounding():
    def __init__(self, max_force):
        self.max_force = max_force
    
    def check_force(self, force):
        # turn x y z force into magnitude
        total_force = (force[0]**2 + force[1]**2 + force[2]**2)**0.5
        if (total_force < self.max_force):
            return True
        else:
            print("Force out of bounds")
            return False

class SafetyHelper():
    def __init__(self, bounding_box, velocity_bounding, acceleration_bounding, force_bounding):
        self.bounding_box = bounding_box
        self.velocity_bounding = velocity_bounding
        self.acceleration_bounding = acceleration_bounding
        self.force_bounding = force_bounding

    def check_safety(self, position, velocity, acceleration, force):
        if (self.bounding_box.check_position(position) and
                self.velocity_bounding.check_velocity(velocity) and
                self.acceleration_bounding.check_acceleration(acceleration) and
                self.force_bounding.check_force(force)):
            return True
        else:
            return False
        
    # cap accceleration to max_acceleration alpha = [ddx ddy ddz ddphi ddtheta ddpsi]
    def cap_alpha(self, alpha):
        total_acc = (alpha[0]**2 + alpha[1]**2 + alpha[2]**2)**0.5
        total_ang_acc = (alpha[3]**2 + alpha[4]**2 + alpha[5]**2)**0.5

        
        if total_acc > self.acceleration_bounding.max_acceleration:
            alpha[0] = alpha[0] * self.acceleration_bounding.max_acceleration / total_acc
            alpha[1] = alpha[1] * self.acceleration_bounding.max_acceleration / total_acc
            alpha[2] = alpha[2] * self.acceleration_bounding.max_acceleration / total_acc
        
        if total_ang_acc > self.acceleration_bounding.max_ang_acceleration:
            alpha[3] = alpha[3] * self.acceleration_bounding.max_ang_acceleration / total_ang_acc
            alpha[4] = alpha[4] * self.acceleration_bounding.max_ang_acceleration / total_ang_acc
            alpha[5] = alpha[5] * self.acceleration_bounding.max_ang_acceleration / total_ang_acc 

        return alpha
