'''
for the Raspberry Pi 3 side
'''
import socket
import json
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

def start_server(host='0.0.0.0', port=65432):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server listening on {host}:{port}")
        while True:  # Continuously accept connections
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    print(f"Received: {data.decode()}")
                    angles = process_received_data(data)
                    control_servos(angles)

def process_received_data(data):
    # Convert received data to a list of angles
    # Assuming data is sent as a comma-separated string of angles
    angles_str = data.decode()  # Convert bytes to string
    angles_obj = json.loads(angles_str)  # Deserialize the JSON string to a Python object
    
    return angles_obj

def release():
    for i in range(15):
        kit.servo[i].angle = None


def control_servos(angles):
    print(f"Received: {angles}")

    # map the angles to the real hardware
    # coxa and tibia move from -80° to +80°
    # femur moves from 0° to 150° 
    # incomming angles are -90° to + 90° for coxa and tibia
    # and  0° to 180°
  
    
    angle      =  [[angles[0][0],  angles[0][1], angles[0][2]], # RR
                   [angles[1][0],  angles[1][1], angles[1][2]], # RF 
                   [angles[2][0], -angles[2][1],-angles[2][2]], # LF
                   [angles[3][0], -angles[3][1],-angles[3][2]]] # LR
                    
    corrections =  [[+10, +0, +0], # RR
                    [+10, +0, +0], # RF 
                    [ +10,-7, +5], # LF
                    [ +4, +0, +15]] # LR

    rotations =    [[ 1, 1, -1], # RR
                    [-1, 1, -1], # RF 
                    [ 1,  -1, -1], # LF
                    [-1, 1, +1]] # LR

    
    kit.servo[3].angle = 90 +   rotations[0][0] *  (angle[0][0] + rotations[0][0]*corrections[0][0])  # RR coxa
    #time.sleep(0.001)
    kit.servo[5].angle = 90 +   rotations[0][1] *  (angle[0][1] + rotations[0][1]*corrections[0][1])  # RR tibia
    #time.sleep(0.001)
    kit.servo[6].angle =  180 - (angle[0][2] -rotations[0][2]*corrections[0][2]) # RR femur
    #time.sleep(0.001)
    kit.servo[8].angle = 90 +   rotations[1][0] *  (angle[1][0] + rotations[1][0]*corrections[1][0])  # RF coxa
    #time.sleep(0.001)
    kit.servo[9].angle = 90 +   rotations[1][1] *  (angle[1][1] + rotations[1][1]*corrections[1][1])  # RF tibia
    #time.sleep(0.001)
    kit.servo[10].angle = 180 - (angle[1][2] -rotations[1][2]*corrections[1][2]) # RF femur
    #time.sleep(0.001)
    kit.servo[0].angle = 90 +   rotations[2][0] *  (angle[2][0] + rotations[2][0]*corrections[2][0]) # LF coxa
    #time.sleep(0.001)
    kit.servo[1].angle = 90 +   rotations[2][1] *  (angle[2][1] + rotations[2][1]*corrections[2][1]) # LF tibia
    #time.sleep(0.001)
    kit.servo[2].angle =  180 - (angle[2][2] -rotations[2][2]*corrections[2][2]) # LF femur
    #time.sleep(0.001)
    kit.servo[12].angle = 90 + rotations[3][0] *   (angle[3][0] + rotations[3][0]*corrections[3][0]) # LR coxa
    #time.sleep(0.001)
    kit.servo[13].angle = 90 + rotations[3][1] *   (angle[3][1] + rotations[3][1]*corrections[3][1]) # LR tibia
    #time.sleep(0.001)
    kit.servo[14].angle =  0 + (angle[3][2] + rotations[3][2]*corrections[3][2]) # LR femur
    #time.sleep(0.001)
if __name__ == "__main__":
    start_server()
