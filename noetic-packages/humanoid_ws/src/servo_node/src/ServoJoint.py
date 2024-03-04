class ServoJoint:
    def __init__(self):
        self.setpoint = 150.0
        self.position = 150.0
        self.velocity = 0.0
        self.load = 0.0
        
    def setAngle(self, setpoint):
        self.setpoint = setpoint
        
    def storeFeedback(self, fb_arr):
        self.position = fb_arr[0]
        self.velocity = fb_arr[1]
        self.load = fb_arr[2]
        
    def getPosition(self):
        return self.position
    
    def getVelocity(self):
        return self.velocity

    def getLoad(self):
        return self.load
    
    def getSetpoint(self):
        return self.setpoint