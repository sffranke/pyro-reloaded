import socket
import json
import logging
from adafruit_servokit import ServoKit

# Initialize the ServoKit instance

kit = ServoKit(channels=16)

def configure_logging():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def start_server(host='0.0.0.0', port=65432):
    '''Starts the server to listen for incoming connections.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        logging.info(f"Server listening on {host}:{port}")
        while True:
            conn, addr = s.accept()
            handle_connection(conn, addr)

def handle_connection(conn, addr):
    '''Handles a new client connection.'''
    logging.info(f"Connected by {addr}")
    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                release_servos()
                break
            logging.info(f"Received: {data.decode()}")
            angles = process_received_data(data)
            control_servos(angles)

def process_received_data(data):
    '''Processes the received data and converts it to a list of angles.'''
    try:
        angles_str = data.decode()
        angles_obj = json.loads(angles_str)
        return angles_obj
    except json.JSONDecodeError as e:
        logging.error(f"Error decoding JSON: {e}")
        return []

def release_servos():
    '''Releases all servos by setting their angles to None.'''
    for i in range(16):
        kit.servo[i].angle = None

def control_servos(angles):
    '''Controls the servos based on the received angles.'''
    if len(angles) != 4:
        logging.error("Invalid number of angle sets received. Expected 4 sets of angles.")
        return

    angle = [
        [angles[0][0], angles[0][1], angles[0][2]],  # RR
        [angles[1][0], angles[1][1], angles[1][2]],  # RF 
        [angles[2][0], -angles[2][1], -angles[2][2]],  # LF
        [angles[3][0], -angles[3][1], -angles[3][2]]  # LR
    ]
    
    corrections = [
        [+10, +0, +0],  # RR
        [-10, +0, +0],  # RF 
        [+10, -7, +5],  # LF
        [-4, +0, +15]  # LR
    ]
    
    rotations = [
        [1, +1, -1],  # RR
        [-1, 1, -1],  # RF 
        [1, -1, -1],  # LF
        [-1, 1, +1]  # LR
    ]
    
    servo_angles = [
        (3, 90 + rotations[0][0] * (angle[0][0] + corrections[0][0])),  # RR coxa
        (5, 90 + rotations[0][1] * (angle[0][1] + corrections[0][1])),  # RR tibia
        (6, 180 - (angle[0][2] - corrections[0][2])),  # RR femur
        (8, 90 + rotations[1][0] * (angle[1][0] + corrections[1][0])),  # RF coxa
        (9, 90 + rotations[1][1] * (angle[1][1] + corrections[1][1])),  # RF tibia
        (10, 180 - (angle[1][2] - corrections[1][2])),  # RF femur
        (0, 90 + rotations[2][0] * (angle[2][0] + corrections[2][0])),  # LF coxa
        (1, 90 + rotations[2][1] * (angle[2][1] + corrections[2][1])),  # LF tibia
        (2, 180 - (angle[2][2] - corrections[2][2])),  # LF femur
        (12, 90 + rotations[3][0] * (angle[3][0] + corrections[3][0])),  # LR coxa
        (13, 90 + rotations[3][1] * (angle[3][1] + corrections[3][1])),  # LR tibia
        (14, angle[3][2] + corrections[3][2])  # LR femur
    ]

    for servo, angle in servo_angles:
        kit.servo[servo].angle = angle
        #logging.info(f"Set servo {servo} to angle {angle}")

if __name__ == "__main__":
    configure_logging()
    start_server()
