import socket
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import math
import numpy as np
import networkx as nx

IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}
nodes = {}
orientation_error = []


world_nodes = {1: [-1.204, -0.142],
               2: [-0.264, -0.289],
               3: [0.749, -0.200],
               4: [0.818,  1.314],
               5: [-0.165, 1.377],
               6: [-0.287, 0.925],
               7: [-0.177, 0.283],
               8: [-0.794, 0.827],
               9: [-1.388, 0.189],
               10: [0.940, 0.291]}



# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
   
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

def run(positions, robot_id, i):

    # Convert the nodes into their x and y coordinates
    xd = world_nodes[nodes[i]][0]
    yd =  world_nodes[nodes[i]][1]
    while True:
        # Distance to goal
        x_curr = positions[robot_id][0]
        y_curr = positions[robot_id][1]
        distToGoal = math.sqrt((xd - x_curr)**2 + (yd - y_curr)**2)

        # Compute the desired orientation
        alpha = math.atan2((yd - y_curr), (xd - x_curr))
        theta = math.radians(rotations[robot_id]) 

        kw = 200
        # omega = (alpha - theta) * kw
        w = kw * math.degrees(math.atan2(math.sin(alpha - theta), math.cos(alpha - theta)))
        print(nodes[i])
        # proportional-controller
        k = 1900
        v = distToGoal * k

        # print(str(distToGoal) + " i: " +str(i))
        # position-controller
        u = np.array([v - w, v + w])
        u[u > 1500] = 1500
        u[u < -1500] = -1500
        # Send control input to the motors
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])    
        s.send(command.encode('utf-8'))
        time.sleep(.1)
    #  distances.append(distToGoal)
        if(distToGoal < 0.2):
            break

if __name__ == "__main__":
    try:

        clientAddress = "192.168.0.182"
        optitrackServerAddress = "192.168.0.172"
        robot_id = 300

         # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        is_running = streaming_client.run()
        # P controller for angle

        # Calculate the shortest path and make an array out of it

        start_node = 2
        end_node = 8


        G = nx.Graph()
        G.add_nodes_from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        G.add_edges_from([(1, 2), (2, 3), (3, 7), (7, 10), (7,6), (3, 10), (10, 4), (4,5), (5,6), (6, 8), (8,9), (9,1), (2,7)])


        nodes = nx.dijkstra_path(G, start_node, end_node)


        while is_running:
            if robot_id in positions:
                for i in range(len(nodes)):
                    run(positions, robot_id, i)
                    time.sleep(.3)
                break
        


    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()
        sys.exit()
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    print("Connection closed!")
    s.shutdown(2)
    s.close()
    sys.exit()


# there are a few parts of the algorithm
# finding the shortest path
# loading these values into some kind of array
# following these points the same way I would the square

# How to create


# If I just had two points0000000


# Coordinates
#       xXXXXX      yYYYYY
# 1     -1.204,      -0.142 
# 2     -0.264,     -0.289
# 3     0.749,       -.200
# 4     0.818,       1.314
# 5     -0.165,      1.377
# 6     -0.287,      0.925
# 7     -0.177,      0.283
# 8     -0.794,      0.827
# 9     -1.388,      0.189
# 10    0.940,       0.291
