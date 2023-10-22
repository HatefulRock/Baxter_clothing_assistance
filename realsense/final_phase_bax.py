# we have the 3 points shoulder, elbow and wrist for each arm
# we then have the vector in 3D space towards where the next point should be 
# the vector is in the realsense world but in the baxter world should be same vector
# the starting position of the baxter hands should be around the shoulder of the person so we just need to use the first vector to go
# to the elbow and then the second vector to go to the wrist



import argparse
import struct
import sys
import copy

import rospy
import rospkg
import socket
import numpy as np
from rospy import Duration
import threading

from std_msgs.msg import String


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,

)

import baxter_interface
import baxter_examples




import argparse
import sys

import rospy

from baxter_interface import (
    DigitalIO,
    Gripper,
    Navigator,
    CHECK_VERSION,
)



Lvect3DStoE=[]
Lvect3DEtoW=[]
Rvect3DStoE=[]
Rvect3DEtoW=[]

Rshoulder2elbow=True
Lshoulder2elbow=True
Relbow2wrist=True
Lelbow2wrist=True



def Shoulder2Elbow(start_point,vector,distance):
    new_point = [
            start_point[0] + vector[0] * distance,
            start_point[1] + vector[1] * distance,
            start_point[2] + vector[2] * distance
        ]
    return new_point


def Elbow2Wrist(start_point,vector,distance):
    new_point = [
            start_point[0] + vector[0] * distance,
            start_point[1] + vector[1] * distance,
            start_point[2] + vector[2] * distance
        ]
    return new_point


# Define the server address and port
server_address = '192.168.0.21'
server_port = 5353

class GripperConnect(object):
    """
    Connects wrist button presses to gripper open/close commands.

    Uses the DigitalIO Signal feature to make callbacks to connected
    action functions when the button values change.
    """

    def __init__(self, arm, lights=True):
        """
        @type arm: str
        @param arm: arm of gripper to control {left, right}
        @type lights: bool
        @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm
        # inputs
        self._close_io = DigitalIO('%s_upper_button' % (arm,))  # 'dash' btn
        self._open_io = DigitalIO('%s_lower_button' % (arm,))   # 'circle' btn
        self._light_io = DigitalIO('%s_lower_cuff' % (arm,))    # cuff squeeze
        # outputs
        self._gripper = Gripper('%s' % (arm,), CHECK_VERSION)
        self._nav = Navigator('%s' % (arm,))

        # connect callback fns to signals
        if self._gripper.type() != 'custom':
            if not (self._gripper.calibrated() or
                    self._gripper.calibrate() == True):
                rospy.logwarn("%s (%s) calibration failed.",
                              self._gripper.name.capitalize(),
                              self._gripper.type())
        else:
            msg = (("%s (%s) not capable of gripper commands."
                   " Running cuff-light connection only.") %
                   (self._gripper.name.capitalize(), self._gripper.type()))
            rospy.logwarn(msg)

        self._gripper.on_type_changed.connect(self._check_calibration)
        self._open_io.state_changed.connect(self._open_action)
        self._close_io.state_changed.connect(self._close_action)

        if lights:
            self._light_io.state_changed.connect(self._light_action)

        rospy.loginfo("%s Cuff Control initialized...",
                      self._gripper.name.capitalize())

    def _open_action(self, value):
        if value and self._is_grippable():
            rospy.logdebug("gripper open triggered")
            self._gripper.open()

    def _close_action(self, value):
        if value and self._is_grippable():
            rospy.logdebug("gripper close triggered")
            self._gripper.close()

    def _light_action(self, value):
        if value:
            rospy.logdebug("cuff grasp triggered")
        else:
            rospy.logdebug("cuff release triggered")
        self._nav.inner_led = value
        self._nav.outer_led = value

    def _check_calibration(self, value):
        if self._gripper.calibrated():
            return True
        elif value == 'electric':
            rospy.loginfo("calibrating %s...",
                          self._gripper.name.capitalize())
            return (self._gripper.calibrate() == True)
        else:
            return False

    def _is_grippable(self):
        return (self._gripper.calibrated() and self._gripper.ready())

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()

        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def grip(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        #self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        #self._retract()

    def forward(self,pose):
        self._approach(pose)



    def move(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()



# Create a socket object
#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the server address and port
#sock.bind((server_address, server_port))

# Listen for incoming connections
#sock.listen(1)
#print("Waiting for a connection...")

# Accept a connection from a client
#client_socket, client_address = sock.accept()

# Bind the socket to the server address and port
#sock.bind((server_address, server_port))
# Create a socket and connect to the server
#sock.connect((server_address, server_port))
#midpoints_ready = threading.Event()  # Event to indicate when 'midpoints' is ready

def callback(data):
    # This function will be called whenever a new message is received on the subscribed topic
    #rospy.loginfo("Received message: %s", data.data)
    # Process the received message here
    midpoints_str = data.data
    midpoints=[float(coordinate.strip()) for coordinate in midpoints_str.split(', ')]
    #midpoints_ready.set()  # Set the event to indicate that 'midpoints' is ready

    #print("Midpoints:", midpoints)

def listener():
    rospy.init_node('message_reader')  # Initialize the ROS node
    rospy.Subscriber('midpoints_topic', String, callback)  # Subscribe to the desired topic
    #rospy.spin()  # Keep the node running until it is explicitly shut down

def transform_point_l_arm(point):
    vectL=[0.224,-0.177,0.087]
    point[0]=point[0]+vectL[0]
    point[1]=point[1]+vectL[1]
    point[2]=point[2]+vectL[2]
    return point

def transform_point_r_arm(point):
    vectR=[0.285,-0.054,0.1]
    point[0]=point[0]+vectR[0]
    point[1]=point[1]+vectR[1]
    point[2]=point[2]+vectR[2]
    return point


def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")

    # Create a thread for the listener function
    #listener()
    server_address = '192.168.0.21'
    server_port = 5353

    rospy.sleep(Duration(9.0))
    #print("Midpoints:", midpoints)


    # Continue with the rest of your program logic
    print("Running the rest of the program...")
    #print("Midpoints",midpoints)
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    #rospy.wait_for_message("/robot/sim/started", Empty)
    #data = client_socket.recv(1024)

    #if not data:
        # Connection closed by the client
        #print("Nothing received")

    limbl = 'left'
    limbr="right"
    hover_distance = 0.0 # meters
    grip_ctrls1 = GripperConnect(limbl, lights=False)
    grip_ctrls2 = GripperConnect(limbr, lights=False)

    starting_joint_angles_right={'right_s0': 0.6028544496389676, 'right_s1': -1.1700438459595994,
     'right_w0': -3.003150887482669, 'right_w1': -0.7838641826094465, 'right_w2': 1.758708973310627, 'right_e0': -0.16413594430373926, 'right_e1': 2.068189597266509}
    starting_joint_angles_left={'left_w0': 0.20478643518270273, 'left_w1': -0.06596117387907278, 'left_w2': -0.25464081078897866,'left_e0': -0.19213109368264808, 'left_e1': 2.508825578586594, 'left_s0': -0.3305728597893067, 'left_s1': -1.1455001533534328}

    # Starting Joint angles for left arm
    jacket_joint_angles_left = {"left_w0": -0.37697577862284043, 'left_w1': -0.8885583713826259, 'left_w2': 0.11543205428837738, 'left_e0': -0.008053399136398421,
     'left_e1': 2.154476016585064, 'left_s0': -1.0461748973378522, 'left_s1': -0.8640146787764593}


    jacket_joint_angles_right = {"right_s0": 1.2298690966871304, 'right_s1': -0.724805922275858,
     'right_w0': -2.971704281331018, 'right_w1': 1.14473316295949, 'right_w2': 3.0353644840282623, 'right_e0': -0.19251458887961942, 'right_e1': 2.1245633912212982}


    #the arms of the robot are at shoulder level and the person is facing away from the robot
    #we use the vectors to help us calculate the direction in which the robot needs to move in
    #we have a vector shoulder to elbow and a vector elbow to wrist
    pnpl = PickAndPlace(limbl, hover_distance)
    pnpr= PickAndPlace(limbr, hover_distance)
    overhead_orientation = Quaternion(
                                 x=-0.0249590815779,
                                 y=0.5,
                                 z=-0.0,
                                 w=0.4)

    while not rospy.is_shutdown():

        while Rshoulder2elbow and Lshoulder2elbow:
            S2Eposesl = list()
            S2Eposesr= list()

            distance=20#distance from shoulder 2 elbow
            actual_coords_l=[]#starting coords of the arms
            actual_coords_r=[]#starting coords of the arms
            for i in range(1,distance,5):
                actual_coords_l=Shoulder2Elbow(actual_coords_l,Lvect3DStoE,5)
                actual_coords_r=Shoulder2Elbow(actual_coords_r,Rvect3DStoE,5)

                S2Eposesl.append(Pose(
                position=Point(x=actual_coords_l[0], y=actual_coords_l[1], z=actual_coords_l[2]),
                orientation=overhead_orientation))

                S2Eposesr.append(Pose(
                position=Point(x=actual_coords_r[0], y=actual_coords_r[0], z=actual_coords_r[0]),
                orientation=overhead_orientation))

                pnpl.forward(S2Eposesl[-1])
                pnpr.forward(S2Eposesr[-1])

            
            Rshoulder2elbow=False
            Lshoulder2elbow=False
        
        while Relbow2wrist and Lelbow2wrist:

            E2Wposesl = list()
            E2Wposesr= list()

            distance=20#distance from shoulder 2 elbow
            actual_coords_l=[]#starting coords of the arms
            actual_coords_r=[]#starting coords of the arms
            for i in range(1,distance,5):
                actual_coords_l=Shoulder2Elbow(actual_coords_l,Lvect3DStoE,5)
                actual_coords_r=Shoulder2Elbow(actual_coords_r,Rvect3DStoE,5)

                E2Wposesl.append(Pose(
                position=Point(x=actual_coords_l[0], y=actual_coords_l[1], z=actual_coords_l[2]),
                orientation=overhead_orientation))

                E2Wposesr.append(Pose(
                position=Point(x=actual_coords_r[0], y=actual_coords_r[0], z=actual_coords_r[0]),
                orientation=overhead_orientation))

                pnpl.forward(E2Wposesl[-1])
                pnpr.forward(E2Wposesr[-1])

            
            Rshoulder2elbow=False
            Lshoulder2elbow=False
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())
