import time
from avp_stream import VisionProStreamer
from avp_leap import Leapv1PybulletIKPython
from leap_hand_api.leap_hand_utils import LeapNode

'''
This is based off of https://github.com/Improbable-AI/VisionProTeleop by Younghyo Park et. al. which streams the AVP data.
We then runs inverse kinematics and output LEAP Hand joint angles.
Feed these joint angles into your preferred LEAP Hand API, Python, ROS1/ROS2 etc.

See https://github.com/leap-hand/Bidex_VisionPro_Teleop for further details.

Note how the fingertip positions are matching, but the joint angles between the two hands are not due to the IK solution.  :) 
This is not as accurate as Manus gloves but easier to use.
Inspired by Dexcap https://dex-cap.github.io/ by Wang et. al. and Robotic Telekinesis by Shaw et. al.
'''

#Set this to your IP address of you AVP
AVP_IP = "172.30.1.98"  # RLWRLD robot lab

if __name__ == "__main__":
    leap_hand = LeapNode()
    vps = VisionProStreamer(ip = AVP_IP, record = True)
    pbik = Leapv1PybulletIKPython(vps, is_left=False)

    while True:
        ##This is the 16 dimensional LEAP Hand vector  Feed this into your python or ROS1/ROS2 code.  Ordering is the same!
        output_joints = pbik.get_avp_data()
        print("Vision pro pos: " + str(output_joints))

        leap_hand.set_allegro(output_joints)
        print("Actual pos: " + str(leap_hand.read_pos()))

        # Make it slow to be safe
        time.sleep(0.03)