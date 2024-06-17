

class JointServoPD: 
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0

    def update(self, error, dt):
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.kd * derivative
    
    