import math
import time
import logging
import csv


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/5/2M/EE5C21CF04')

# Parmeters set in loggin function
initial_position = [0,0,0]

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
        offset_x, offset_y = waypoints[0][0], waypoints[0][1]
        normalized_waypoints = []
        for point in waypoints:
            # Convert to tuples with height 0.4
            normalized_waypoints.append(((point[0] - offset_x)/10, -(point[1] - offset_y)/10, 0.4))
        return normalized_waypoints
    
    return waypoints

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(15):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def log_initial_pos(scf, logconf):

    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            initial_position[0] = data['stateEstimate.x']
            initial_position[1] = data['stateEstimate.y']
            initial_position[2] = data['stateEstimate.z']

            break

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_yaw = 90  # In degrees

    lg_stab = LogConfig(name='pos', period_in_ms=2000)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        log_initial_pos(scf, lg_stab)


    print("Initial position: ", initial_position[0], initial_position[1], initial_position[1])
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        set_initial_position(scf, initial_position[0], initial_position[1], initial_position[2], initial_yaw)
        reset_estimator(scf)
        sequence = read_path_waypoints('path.csv')
        run_sequence(scf, sequence, initial_position[0], initial_position[1], initial_position[2], initial_yaw)
