import math
import time
import logging
import csv

from threading import Event
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/5/2M/EE5C21CF04')

flow_deck_attached_event = Event()

# Parmeters set in loggin function
estimated_position = [0,0,0]
initial_position = [0,0,0]
shutter_num = 0
shutter_readings = []  # Store last 10 readings for moving average
MAX_READINGS = 10
COLOUR_THRESHOLD = 2250
skip_blocks = False

# Global variables to store offset for coordinate conversion
offset_x = 0
offset_y = 0

def rotate_clock(sequence):
    rotated_sequence = []
    for pos in sequence:
        x = pos[0]
        y = pos[1]
        z = pos[2]
        new_x = y
        new_y = -x
        rotated_sequence.append((new_x, new_y, z))
    return rotated_sequence

def rotate_counterclock(sequence):
    rotated_sequence = []
    for pos in sequence:
        x = pos[0]
        y = pos[1]
        z = pos[2]
        new_x = -y
        new_y = x
        rotated_sequence.append((new_x, new_y, z))
    return rotated_sequence

def read_path_waypoints(csv_file='path.csv'):
    points = []
    try:
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) >= 2:
                    x, y = float(row[0]), float(row[1])
                    points.append([x, y])
    except FileNotFoundError:
        print(f"CSV file {csv_file} not found")
        return []
    
    if len(points) < 2:
        return points
    
    waypoints = [points[0]]
    
    for i in range(1, len(points) - 1):
        prev_dir = [points[i][0] - points[i-1][0], points[i][1] - points[i-1][1]]
        next_dir = [points[i+1][0] - points[i][0], points[i+1][1] - points[i][1]]
        
        if prev_dir != next_dir:
            waypoints.append(points[i])
    
    waypoints.append(points[-1])

    if waypoints:
        global offset_x, offset_y
        offset_x, offset_y = waypoints[0][0], waypoints[0][1]
        normalized_waypoints = []
        for point in waypoints:
            # Convert to tuples with height 0.4
            normalized_waypoints.append(((point[0] - offset_x)/10, -(point[1] - offset_y)/10, 0.18))
        return normalized_waypoints
    
    return waypoints

def read_all_blocks(csv_file='path.csv'):
    """Read all individual blocks/cells from CSV and return them as a sequence with height 4"""
    blocks = []
    try:
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) >= 2:
                    x, y = float(row[0]), float(row[1])
                    blocks.append([x, y])
    except FileNotFoundError:
        print(f"CSV file {csv_file} not found")
        return []
    
    if not blocks:
        return []
    
    # Normalize blocks similar to waypoints but with height 4
    global offset_x, offset_y
    offset_x, offset_y = blocks[0][0], blocks[0][1]
    normalized_blocks = []
    for block in blocks:
        # Convert to tuples with height 4
        normalized_blocks.append(((block[0] - offset_x)/10, -(block[1] - offset_y)/10, 0.18))
    
    return normalized_blocks

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    initial_position[0] = x
    initial_position[1] = y
    initial_position[2] = z

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        flow_deck_attached_event.set()

def over_obstacle():
    global shutter_num
    return shutter_num > COLOUR_THRESHOLD

    
    # Calculate moving average of last 10 readings
    moving_average = sum(shutter_readings) / len(shutter_readings)
    print(f"Moving average of shutter readings: {moving_average}")
    return moving_average > COLOUR_THRESHOLD

def update_csv(obstacle_position, drone_position):
    """Convert normalized coordinates back to original space and log to obstacles.csv"""
    global offset_x, offset_y
    
    # Convert obstacle position back to original coordinate space
    # Reverse the transformation: multiply by 10, negate y, add offset
    obstacle_x = int(obstacle_position[0] * 10 + offset_x)
    obstacle_y = int(-obstacle_position[1] * 10 + offset_y)
    
    # Convert drone position back to original coordinate space
    drone_x = int(drone_position[0] * 10 + offset_x)
    drone_y = int(-drone_position[1] * 10 + offset_y)
    
    # Write to obstacles.csv file
    try:
        with open('obstacles.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            # Write obstacle position and drone position when detected
            writer.writerow([f"{obstacle_x},{obstacle_y}"])
            writer.writerow([f"{drone_x},{drone_y}"])
            writer.writerow([])  # Empty line for separation
        print(f"Logged obstacle at ({obstacle_x},{obstacle_y}), drone was at ({drone_x},{drone_y})")
    except Exception as e:
        print(f"Error writing to obstacles.csv: {e}")

def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf
    prior_position = None
    obstacle_detected = False

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    for i, position in enumerate(sequence):
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(5):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

        print(f"Reached position {position}")
        
        # Check for obstacle over multiple readings     
        black = 0
        for i in range(50):
            if over_obstacle():
                # print("Over black")
                black = black + 1
            time.sleep(0.01)


        print(f"Black count: {black}")
        if black >= 40:
            print("Obstacle detected!")
 

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def log_stab_callback(timestamp, data, logconf):
    global estimated_position, shutter_num
    estimated_position[0] = data['stateEstimate.x']
    estimated_position[1] = data['stateEstimate.y']
    estimated_position[2] = data['stateEstimate.z']
    shutter_num = data['motion.shutter']
    
    # Add new reading to the list
    
    if timestamp % 2000 == 0:  # Print every 200th callback to reduce output
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_yaw = 90  # In degrees

    logconf = LogConfig(name='pos', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('motion.shutter', 'uint16_t')


    print("Initial position: ", estimated_position[0], estimated_position[1], estimated_position[1])
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                        cb=param_deck_flow)
        simple_log_async(scf, logconf)
        time.sleep(0.1)
        set_initial_position(scf, estimated_position[0], estimated_position[1], estimated_position[2], initial_yaw)
        reset_estimator(scf)
        if skip_blocks:
            sequence = read_path_waypoints('path.csv')
        else:
            sequence = read_all_blocks('path.csv')
        sequence = rotate_clock(sequence)
        run_sequence(scf, sequence, estimated_position[0], estimated_position[1], estimated_position[2], initial_yaw)