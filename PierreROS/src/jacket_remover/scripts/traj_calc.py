import csv
from baxter_pykdl import baxter_kinematics
# baxter_interface - Baxter Python API
import baxter_interface
import rospy


# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter')
# Create an instance of the Baxter kinematics solver
kinematicsr = baxter_kinematics('right')
limbr  = baxter_interface.Limb('right')

kinematicsl = baxter_kinematics('left')
limbl  = baxter_interface.Limb('left')

anglesr = limbr.joint_angles()
anglesl = limbl.joint_angles()
# Open and read the original CSV file
csv_file = open('phase2', 'r')
csv_reader = csv.DictReader(csv_file)

# Create a new CSV file for writing the end effector coordinates
output_csv_file = open('phase2coord.csv', 'w')
fieldnames = ['Left_X', 'Left_Y', 'Left_Z', 'Right_X', 'Right_Y', 'Right_Z']
csv_writer = csv.DictWriter(output_csv_file, fieldnames=fieldnames)
csv_writer.writeheader()


# Iterate through the rows of the original CSV file
for index, row in enumerate(csv_reader):
    # Extract joint angles from the current row
    joint_anglesr = {
        'right_s0': float(row['right_s0']),
        'right_s1': float(row['right_s1']),
        'right_e0': float(row['right_e0']),
        'right_e1': float(row['right_e1']),
        'right_w0': float(row['right_w0']),
        'right_w1': float(row['right_w1']),
        'right_w2': float(row['right_w2']),
    }

    joint_anglesl = {
        'left_s0': float(row['left_s0']),
        'left_s1': float(row['left_s1']),
        'left_e0': float(row['left_e0']),
        'left_e1': float(row['left_e1']),
        'left_w0': float(row['left_w0']),
        'left_w1': float(row['left_w1']),
        'left_w2': float(row['left_w2']),
    }

    # Calculate the end effector positions for both arms
    end_effector_posr = kinematicsr.forward_position_kinematics(joint_anglesr)
    end_effector_posl = kinematicsl.forward_position_kinematics(joint_anglesl)

    # Extract x, y, and z coordinates for both arms
    left_x, left_y, left_z = end_effector_posl[:3]
    right_x, right_y, right_z = end_effector_posr[:3]

    # Write the coordinates to the new CSV file
    csv_writer.writerow({
        'Left_X': left_x,
        'Left_Y': left_y,
        'Left_Z': left_z,
        'Right_X': right_x,
        'Right_Y': right_y,
        'Right_Z': right_z,
    })


    # Print the end effector positions for the current row
    #print("Row {0}:".format(index + 1))
    #print("Right End Effector Position:", end_effector_posr)
    #print("Left End Effector Position:", end_effector_posl)
    #print("\n")

# Close both CSV files
csv_file.close()
output_csv_file.close()
