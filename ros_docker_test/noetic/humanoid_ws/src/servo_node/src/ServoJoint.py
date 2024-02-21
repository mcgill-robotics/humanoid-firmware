class ServoJoint:
    def __init__(self, id=0, name="default_name"):
        self.id = id
        self.name = name
        self.setpoint_raw = 0
        self.pos_raw = 0
        self.is_faulted = False

    def get_pos_deg(self):
        pos_deg = self.pos_raw * 360 / 1023
        return pos_deg

    def get_setpoint_deg(self):
        setpoint_deg = self.setpoint_raw * 360 / 1023
        return setpoint_deg

    def set_pos_deg(self, pos_deg):
        self.pos_raw = pos_deg * 1023 / 360

    def set_setpoint_deg(self, setpoint_deg):
        self.setpoint_raw = setpoint_deg * 1023 / 360
