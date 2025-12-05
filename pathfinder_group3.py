## @file pathfinder_group3.py
#  @brief Drone path finding and obstacle avoidance system using Crazyflie
#  @details This module provides autonomous flight capabilities for a Crazyflie drone,
#           including path planning, obstacle detection, and coordinate system management.
#  @author Group 3
#  @date December 2025

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

## @brief URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/5/2M/EE5C21CF04')

## @brief Event to track flow deck attachment status
flow_deck_attached_event = Event()

## @brief Current estimated position [x, y, z] used in logging
estimated_position = [0,0,0]

## @brief Initial position [x, y, z] used for Kalman filter initialization
initial_position = [0,0,0]

## @brief Current shutter reading from motion sensor
shutter_num = 0

## @brief Threshold for how many black readings to be over obstacle
MAX_READINGS = 10

## @brief Threshold for shutter reading to consider surface as black/obstacle
COLOUR_THRESHOLD = 3500

## @brief If true, read waypoints instead of all blocks (won't avoid obstacles)
skip_blocks = False

## @brief Global X offset for coordinate conversion from original to normalized space
offset_x = 0

## @brief Global Y offset for coordinate conversion from original to normalized space
offset_y = 0

## @brief Rotates the drone path about the Z axis
#  @details Used to correct when the mat is not aligned with global lighthouse coordinates
#  @param sequence List of position tuples (x, y, z) to rotate
#  @param theta Rotation angle in radians
#  @return List of rotated position tuples
def rotate_sequence(sequence, theta):
    rotated_sequence = []
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    for pos in sequence:
        x = pos[0]
        y = pos[1]
        z = pos[2]
        new_x = x * cos_theta - y * sin_theta
        new_y = x * sin_theta + y * cos_theta
        rotated_sequence.append((new_x, new_y, z))
    return rotated_sequence

## @brief Reads path waypoints from MATLAB CSV and returns a simplified sequence
#  @details Sequence contains only turning points and start/end points.
#           Can only detect new obstacles at corners and will miss obstacles in straight paths.
#  @param csv_file Path to the CSV file containing waypoint data
#  @return List of normalized waypoint tuples (x, y, z)
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
            normalized_waypoints.append(((point[0] - offset_x)/10, -(point[1] - offset_y)/10, 0.15))
        return normalized_waypoints
    
    return waypoints

## @brief Reads all blocks from MATLAB CSV and returns them as a sequence
#  @details Allows for drone to stop at each block and detect new obstacles
#  @param csv_file Path to the CSV file containing block data
#  @return List of normalized block position tuples (x, y, z)
def read_all_blocks(csv_file='path.csv'):
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
        normalized_blocks.append(((block[0] - offset_x)/10, -(block[1] - offset_y)/10, 0.15))
    
    return normalized_blocks

## @brief Sets the initial position for the Kalman filter
#  @param scf SyncCrazyflie object
#  @param x Initial X coordinate
#  @param y Initial Y coordinate
#  @param z Initial Z coordinate
#  @param yaw_deg Initial yaw angle in degrees
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    initial_position[0] = x
    initial_position[1] = y
    initial_position[2] = z

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

## @brief Checks if flow deck is attached
#  @param _ Unused parameter (parameter name)
#  @param value_str String value indicating flow deck status
def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        flow_deck_attached_event.set()

## @brief Checks if the drone is over an obstacle based on shutter reading
#  @return True if shutter reading exceeds COLOUR_THRESHOLD, False otherwise
def over_obstacle():
    global shutter_num
    return shutter_num > COLOUR_THRESHOLD

## @brief Updates the obstacles.csv file with detected obstacle position
#  @details This CSV can then be read by MATLAB to update the path.
#           Converts normalized coordinates back to original space.
#  @param obstacle_position Normalized position tuple (x, y, z) of detected obstacle
def update_csv(obstacle_position):
    global offset_x, offset_y
    
    # Convert obstacle position back to original coordinate space
    # Reverse the transformation: multiply by 10, negate y, add offset
    obstacle_x = int(obstacle_position[0] * 10 + offset_x)
    obstacle_y = int(-obstacle_position[1] * 10 + offset_y)
    
    
    # Write to obstacles.csv file
    try:
        with open('obstacles.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([obstacle_x, obstacle_y])
        print(f"Logged obstacle at ({obstacle_x},{obstacle_y}))")
    except Exception as e:
        print(f"Error writing to obstacles.csv: {e}")

## @brief Flies the drone through a given sequence of positions
#  @details Checks for obstacles at each position and logs them if detected.
#           Sets starting point as (0,0,0) and moves relative to that point instead of global lighthouse coordinates.
#           Takes approximately 0.5 seconds to move from point to point.
#  @param scf SyncCrazyflie object
#  @param sequence List of position tuples to fly through
#  @param base_x Base X coordinate offset
#  @param base_y Base Y coordinate offset
#  @param base_z Base Z coordinate offset
#  @param yaw Yaw angle in degrees
def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf
    obstacle_detected = False

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    for i, position in enumerate(sequence):
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for j in range(10):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

        print(f"Reached position {position}")
        
        ## @brief Check for obstacle over multiple readings to reduce false positives
        black = 0
        for j in range(50):
            if over_obstacle():
                black = black + 1
            time.sleep(0.01)


        print(f"Black count: {black}")
        if black >= 40 and i != 0:
            for j in range(30):
                cf.commander.send_position_setpoint(initial_position[0], initial_position[1], 0.15, yaw)
                time.sleep(0.1)
            obstacle_detected = True
            update_csv(position)
            print("Obstacle detected!")

        if obstacle_detected:
            break


    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

## @brief Logging callback function that updates estimated position and shutter reading
#  @details These variables are needed to determine what direction to move and if over obstacle.
#           Callback is called every 10ms as per log config.
#  @param timestamp Current timestamp
#  @param data Dictionary containing logged data values
#  @param logconf Log configuration object
def log_stab_callback(timestamp, data, logconf):
    global estimated_position, shutter_num
    estimated_position[0] = data['stateEstimate.x']
    estimated_position[1] = data['stateEstimate.y']
    estimated_position[2] = data['stateEstimate.z']
    shutter_num = data['motion.shutter']
        
    if timestamp % 2000 == 0:  # Print every 200th callback to reduce output
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))

## @brief Initializes logging with the given log configuration
#  @param scf SyncCrazyflie object
#  @param logconf Log configuration object to start
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

## @brief Main program entry point
#  @details Main program logic:
#           1. Sets up logging
#           2. Sets up initial position
#           3. Reads path from MATLAB CSV (chooses waypoints or all blocks)
#           4. Rotates path if needed
#           5. Runs the sequence
#           6. Prompts user to run again or exit
if __name__ == '__main__':
    cflib.crtp.init_drivers()


    initial_yaw = 90 # In degrees

    logconf = LogConfig(name='pos', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('motion.shutter', 'uint16_t')

    while True:
        print("Initial position: ", estimated_position[0], estimated_position[1], estimated_position[1])
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)
            simple_log_async(scf, logconf)
            time.sleep(0.1)
            set_initial_position(scf, estimated_position[0], estimated_position[1], estimated_position[2], initial_yaw)
            reset_estimator(scf)
            
            if skip_blocks:
                sequence = read_path_waypoints('path.csv')
            else:
                sequence = read_all_blocks('path.csv')
            
            rotate_sequence(sequence, math.radians(0))
            run_sequence(scf, sequence, estimated_position[0], estimated_position[1], estimated_position[2], initial_yaw)
        
        while True:
            response = input("Run the sequence again? (y/n): ").lower().strip()
            if response in ['y', 'yes']:
                break
            elif response in ['n', 'no']:
                print("Exiting program.")
                exit()
            else:
                print("Please enter 'y' for yes or 'n' for no.")



                ## @todo Update firmware, remove the restart yes/no, have it land nicely at the start so it can find path well
                ## @todo Clear obstacle.csv before starting new run