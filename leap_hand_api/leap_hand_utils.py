import numpy as np
from .dynamixel_client import DynamixelClient

'''
Embodiments:

LEAPhand: Real LEAP hand (180 for the motor is actual zero)
LEAPsim:  Leap hand in sim (has allegro-like zero positions)
one_range: [-1, 1] for all joints to facilitate RL
allegro:  Allegro hand in real or sim
'''

class LeapNode:
    def __init__(self):
        ####Some parameters
        # I recommend you keep the current limit from 350 for the lite, and 550 for the full hand
        # Increase KP if the hand is too weak, decrease if it's jittery.
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350  ##set this to 550 if you are using full motors!!!!
        self.prev_pos = self.pos = self.curr_pos = allegro_to_LEAPhand(np.zeros(16))

        # NOTE: check dynamixel wizard
        self.baud_rate = 2_000_000

        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', self.baud_rate)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', self.baud_rate)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', self.baud_rate)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility joint angles.  It adds 180 to make the fully open position at 0 instead of 180
    def set_allegro(self, pose):
        pose = allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility for policies, it assumes the ranges are [-1,1] and then convert to leap hand ranges.
    def set_ones(self, pose):
        pose = sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position of the robot
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    #These combined commands are faster FYI and return a list of data
    def pos_vel(self):
        return self.dxl_client.read_pos_vel()
    #These combined commands are faster FYI and return a list of data
    def pos_vel_eff_srv(self):
        return self.dxl_client.read_pos_vel_cur()


#Safety clips all joints so nothing unsafe can happen. Highly recommend using this before commanding
def angle_safety_clip(joints):
    sim_min, sim_max = LEAPsim_limits()
    real_min = LEAPsim_to_LEAPhand(sim_min)
    real_max = LEAPsim_to_LEAPhand(sim_max)
    return np.clip(joints, real_min, real_max)

###Sometimes it's useful to constrain the thumb more heavily(you have to implement here), but regular usually works good.
def LEAPsim_limits(type = "regular"):
    if type == "regular":
        sim_min = np.array([-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -0.349, -0.47, -1.20, -1.34])
        sim_max = np.array([1.047,    2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  2.094,  2.443, 1.90,  1.88])
    return sim_min, sim_max

#this goes from [-1, 1] to [lower, upper]
def scale(x, lower, upper):
    return (0.5 * (x + 1.0) * (upper - lower) + lower)
#this goes from [lower, upper] to [-1, 1]
def unscale(x, lower, upper):
    return (2.0 * x - upper - lower)/(upper - lower)

#-----------------------------------------------------------------------------------
#Isaac has custom ranges from -1 to 1 so we convert that to LEAPHand real world
def sim_ones_to_LEAPhand(joints, hack_thumb = False):
    sim_min, sim_max = LEAPsim_limits(type = hack_thumb)
    joints = scale(joints, sim_min, sim_max)
    joints = LEAPsim_to_LEAPhand(joints)
    return joints
#LEAPHand real world to Isaac has custom ranges from -1 to 1
def LEAPhand_to_sim_ones(joints, hack_thumb = False):  
    joints = LEAPhand_to_LEAPsim(joints)
    sim_min, sim_max = LEAPsim_limits(type = hack_thumb)
    joints = unscale(joints, sim_min, sim_max)
    return joints

#-----------------------------------------------------------------------------------
###Sim LEAP hand to real leap hand  Sim is allegro-like but all 16 joints are usable.
def LEAPsim_to_LEAPhand(joints):
    joints = np.array(joints)
    ret_joints = joints + 3.14159
    return ret_joints
###Real LEAP hand to sim leap hand  Sim is allegro-like but all 16 joints are usable.
def LEAPhand_to_LEAPsim(joints):
    joints = np.array(joints)
    ret_joints = joints - 3.14159
    return ret_joints

#-----------------------------------------------------------------------------------
#Converts allegrohand radians to LEAP (radians)
#Only converts the joints that match, all 4 of the thumb and the outer 3 for each of the other fingers
#All the clockwise/counterclockwise signs are the same between the two hands.  Just the offset (mostly 180 degrees off)
def allegro_to_LEAPhand(joints, teleop = False, zeros = True):
    joints = np.array(joints)
    ret_joints = joints + 3.14159
    if zeros:
        ret_joints[0] = ret_joints[4] = ret_joints[8] = 3.14
    if teleop:
        ret_joints[12] = joints[12] + 0.2 
        ret_joints[14] = joints[14] - 0.2   
    return ret_joints
# Converts LEAP to allegrohand (radians)
def LEAPhand_to_allegro(joints, teleop = False, zeros = True):
    joints = np.array(joints)
    ret_joints = joints - 3.14159
    if zeros:
        ret_joints[0] = ret_joints[4] = ret_joints[8] = 0
    if teleop:
        ret_joints[12] = joints[12] - 0.2
        ret_joints[14] = joints[14] + 0.2    
    return ret_joints
#-----------------------------------------------------------------------------------
