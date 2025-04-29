class RobotState:
    def __init__(self):
        self.speed = 0
        self.steering_angle = 0
        self.is_auto_mode = False
        self.is_connected = False
        self.camera_active = False
        self.lidar_data = []
        self.proximity_alert = False
        self.sp32port = 'COM5'
        self.lidarport = 'COM6'
        self.autostop = False