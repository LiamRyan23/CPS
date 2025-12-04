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

class CrazyflieMapCalibrator:
    def __init__(self):
        # 4 corner calibration points - UPDATE these with cfclient measurements
        self.lighthouse_coords = {
            (2, 15):  (-0.31, -0.18),   # Start (your observed)
            (2, 2):   (-0.31, 1.12),    # UPDATE: Bottom-left corner
            (15, 2):  (1.00, 1.12),     # UPDATE: Bottom-right corner  
            (15, 15): (1.00, -0.20),    # End (your observed)
        }
        
        self.scale_x = 0.0
        self.scale_y = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.rotation = 0.0
        
        self.calculate_transformation()
    
    def calculate_transformation(self):
        """Calculate transformation using corner points"""
        print("📐 Calculating Crazyflie coordinate transformation...")
        
        # Get corner coordinates
        start_grid, start_real = (2, 15), self.lighthouse_coords[(2, 15)]
        bottom_left_grid, bottom_left_real = (2, 2), self.lighthouse_coords[(2, 2)]
        bottom_right_grid, bottom_right_real = (15, 2), self.lighthouse_coords[(15, 2)]
        end_grid, end_real = (15, 15), self.lighthouse_coords[(15, 15)]
        
        # Calculate scale factors using maximum distances
        grid_y_dist = abs(start_grid[1] - bottom_left_grid[1])  # 13 units
        real_y_dist = abs(start_real[1] - bottom_left_real[1])
        self.scale_y = real_y_dist / grid_y_dist
        
        grid_x_dist = abs(bottom_right_grid[0] - bottom_left_grid[0])  # 13 units
        real_x_dist = abs(bottom_right_real[0] - bottom_left_real[0])
        self.scale_x = real_x_dist / grid_x_dist
        
        # Calculate origin offset
        self.origin_x = start_real[0] - start_grid[0] * self.scale_x
        self.origin_y = start_real[1] - start_grid[1] * self.scale_y
        
        # Calculate rotation
        expected_angle = 0
        actual_angle = math.atan2(bottom_right_real[1] - bottom_left_real[1], 
                                bottom_right_real[0] - bottom_left_real[0])
        self.rotation = actual_angle - expected_angle
        
        print(f"✅ Calibration: X={self.scale_x:.4f}m/grid, Y={self.scale_y:.4f}m/grid, Rot={math.degrees(self.rotation):.1f}°")
    
    def grid_to_crazyflie_coords(self, grid_x, grid_y, height=0.15):
        """Transform grid coordinates to Crazyflie lighthouse coordinates"""
        # Apply scaling
        scaled_x = grid_x * self.scale_x
        scaled_y = grid_y * self.scale_y
        
        # Apply rotation if map is tilted
        if abs(self.rotation) > 0.01:
            cos_r, sin_r = math.cos(self.rotation), math.sin(self.rotation)
            rotated_x = scaled_x * cos_r - scaled_y * sin_r
            rotated_y = scaled_x * sin_r + scaled_y * cos_r
        else:
            rotated_x, rotated_y = scaled_x, scaled_y
        
        # Apply origin offset for lighthouse positioning
        crazyflie_x = rotated_x + self.origin_x
        crazyflie_y = rotated_y + self.origin_y
        
        return (crazyflie_x, crazyflie_y, height)
    
    def crazyflie_to_grid_coords(self, cf_x, cf_y):
        """Reverse transformation: Crazyflie coordinates back to grid"""
        # Remove origin offset
        relative_x = cf_x - self.origin_x
        relative_y = cf_y - self.origin_y
        
        # Reverse rotation
        if abs(self.rotation) > 0.01:
            cos_r, sin_r = math.cos(-self.rotation), math.sin(-self.rotation)
            unrotated_x = relative_x * cos_r - relative_y * sin_r
            unrotated_y = relative_x * sin_r + relative_y * cos_r
        else:
            unrotated_x, unrotated_y = relative_x, relative_y
        
        # Reverse scaling
        grid_x = round(unrotated_x / self.scale_x)
        grid_y = round(unrotated_y / self.scale_y)
        
        return (grid_x, grid_y)

# Global calibrator instance
calibrator = CrazyflieMapCalibrator()

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
            normalized_waypoints.append(((point[0] - offset_x)/10, -(point[1] - offset_y)/10, 0.15))
        return normalized_waypoints
    
    return waypoints

def read_all_blocks(csv_file='path.csv'):
    """Read all individual blocks/cells from CSV and return them as calibrated lighthouse coordinates"""
    blocks = []
    try:
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) >= 2:
                    grid_x, grid_y = int(row[0]), int(row[1])
                    
                    # Convert to Crazyflie lighthouse coordinates using calibration
                    cf_coords = calibrator.grid_to_crazyflie_coords(grid_x, grid_y)
                    blocks.append(cf_coords)
                    
    except FileNotFoundError:
        print(f"CSV file {csv_file} not found")
        return []
    
    if blocks:
        print(f"✅ Loaded {len(blocks)} calibrated waypoints")
    
    return blocks

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

def update_csv(obstacle_position):
    """Convert Crazyflie coordinates back to grid coordinates and log obstacle"""
    
    # Convert from Crazyflie coordinates back to grid coordinates using calibrator
    grid_coords = calibrator.crazyflie_to_grid_coords(obstacle_position[0], obstacle_position[1])
    grid_x, grid_y = grid_coords
    
    # Write to obstacles.csv file
    try:
        with open('obstacles.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([grid_x, grid_y])
        print(f"🚨 Obstacle logged: Grid({grid_x},{grid_y}) from CF({obstacle_position[0]:.3f},{obstacle_position[1]:.3f})")
    except Exception as e:
        print(f"Error writing to obstacles.csv: {e}")

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
        
        # Check for obstacle over multiple readings     
        black = 0
        for j in range(50):
            if over_obstacle():
                # print("Over black")
                black = black + 1
            time.sleep(0.01)


        print(f"Black count: {black}")
        if black >= 45 and i != 0:
            for j in range(30):
                cf.commander.send_position_setpoint(initial_position[0], initial_position[1], 0.2, yaw)
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

    while True:
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
            # sequence = rotate_clock(sequence)
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