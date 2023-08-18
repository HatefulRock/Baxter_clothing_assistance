# we have the 3 points shoulder, elbow and wrist for each arm
# we then have the vector in 3D space towards where the next point should be 
# the vector is in the realsense world but in the baxter world should be same vector
# the starting position of the baxter hands should be around the shoulder of the person so we just need to use the first vector to go
# to the elbow and then the second vector to go to the wrist


Lvect3DStoE=[]
Lvect3DEtoW=[]
Rvect3DStoE=[]
Rvect3DEtoW=[]


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

