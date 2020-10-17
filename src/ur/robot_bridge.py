import urx
from collections import namedtuple
import time
import socket

class RobotBridge():
    
    def __init__(self, ip_address):
        self.device = None
        self.host = ip_address
    
    def __repr__(self):
        return "RobotBridge object on {0}".format(self.host)
    
    def get_status_data(self):
        try:
            status_data = self.device.secmon.get_all_data()['RobotModeData']
            return namedtuple('robot_state', status_data.keys())(*status_data.values())
        except:
            return None
    
    def is_program_running(self):
        status = self.get_status_data()
        if status is not None:
            return status.isProgramRunning
        else:
            return None

    def is_power_on_robot(self):
        status = self.get_status_data()
        if status is not None:
            return status.isPowerOnRobot
        else:
            return None
    
    def is_security_stopped(self):
        status = self.get_status_data()
        if status is not None:
            return status.isSecurityStopped
        else:
            return None

    def is_emergency_stopped(self):
        status = self.get_status_data()
        if status is not None:
            return status.isEmergencyStopped
        else:
            return None
        
    def get_robot_mode(self):
        status = self.get_status_data()
        if status is not None:
            return status.robotMode
        else:
            return None        
    
    def get_pose(self):
        return self.device.get_pose()
    
    def get_tcp(self):
        return self.device.getl()
    
    def is_connected(self, attempts = 3, delay=0.15):
        current_timestamp = 0
        prev_timestamp = 0
        success_count = 0
        for i in range(attempts):
            status = self.get_status_data()
            if status is not None:
                current_timestamp = status.timestamp
                if (current_timestamp - prev_timestamp) > 0:
                    success_count +=1
                prev_timestamp = current_timestamp
            time.sleep(delay)
            if success_count > 1:
#                 print("Connected after {0} iteration".format(i))
                return True
        return False              

    def connect(self, attempts = 5, delay=0.2):
        if self.device is None:
            success_connected = False
            cur_attempt = 0
            while not success_connected and (cur_attempt < attempts):
                try:
                    self.device = urx.Robot(self.host, use_rt=True)
                    time.sleep(delay)
                    success_connected = self.is_connected()
                except urx.ursecmon.TimeoutException:
                    cur_attempt += 1
                except socket.timeout:
                    return False
        else: 
            # TODO destroy threads
            if not self.is_connected():
                self.device = None
                self.connect()
        return self.is_connected()
         
    # motion brdge here    
    def set_motion(self):
        pass