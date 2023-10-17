from baxter_pykdl import baxter_kinematics
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter')
# Create an instance of the Baxter kinematics solver
kinematicsr = baxter_kinematics('right')
limbr  = baxter_interface.Limb('right')

kinematicsl = baxter_kinematics('left')
limbl  = baxter_interface.Limb('left')

anglesr = limbr.joint_angles()
anglesl = limbl.joint_angles()

"""
# Set the joint angles
joint_angles = {
    'right_s0': 1.2298690966871304,
    'right_s1': -0.724805922275858,
    'right_w0': -2.971704281331018,
    'right_w1': 1.14473316295949,
    'right_w2': 3.0353644840282623,
    'right_e0': -0.19251458887961942,
    'right_e1': 2.1245633912212982
}
"""
#joint_angles = {"left_w0": -0.37697577862284043, 'left_w1': -0.8885583713826259, 'left_w2': 0.11543205428837738, 'left_e0': -0.008053399136398421,
 #'left_e1': 2.154476016585064, 'left_s0': -1.0461748973378522, 'left_s1': -0.8640146787764593}
joint_anglesr={'right_s0': 0.8674661355492015, 'right_s1': -0.7044806768363763, 'right_w0': 0.17717478100076528,
'right_w1': -1.2908448330055757, 'right_w2': 2.981675156452273, 'right_e0': 0.18676216092504913, 'right_e1': 2.202412916206483}
joint_anglesl = {"left_w0": -0.37697577862284043, 'left_w1': -0.8885583713826259, 'left_w2': 0.11543205428837738, 'left_e0': -0.008053399136398421,
 'left_e1': 2.154476016585064, 'left_s0': -1.0461748973378522, 'left_s1': -0.8640146787764593}

#for i in range(len(anglesr)):
#    joint_anglesr[i]=anglesr[i]
#    joint_anglesl[i]=anglesl[i]

# Get the current position of the end effector
#print(anglesl)
end_effector_posl = kinematicsl.forward_position_kinematics(anglesl)
end_effector_posr = kinematicsr.forward_position_kinematics(anglesr)


# Print the current position of the end effector
print("Current Right End Effector Position:", end_effector_posr)
print("Current Left End Effector Position:", end_effector_posl)
