import logging
import sys
import time
import argparse
from threading import Event
import numpy as np
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander

# please take note of your drone's channel and address and edit this accordingly
URI = uri_helper.uri_from_env(default='radio://0/5/2M/EE5C21CF04') 
flow_deck_attached_event = Event() # Event for whether the flow deck is attached to the drone

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

shutter_num = 0 # The motion.shutter value read from the flow deck. Used for determining whether a square is white or black
COLOUR_THRESHOLD = 2250 # Threshold for determining whether the drone is above a white or black square


def main():
    cflib.crtp.init_drivers() # Initialise the drivers for the drone

    # Create a synchronous Crazyflie instance using the specified URI
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # Add callbacks for checking whether the Flow Deck and Lighthouse Deck 
        # are attached to the drone
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                        cb=param_deck_flow)

        time.sleep(1)

        # Asynchronously log the x and y position of the drone
        # and the shutter value from the Flow Deck
        logconf = LogConfig(name='', period_in_ms=10)
        logconf.add_variable('motion.shutter', 'uint16_t')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_shutter_callback)

        print("Checking whether flow deck is attached...")
        if not flow_deck_attached_event.wait(timeout=5):
                    print('No flow deck detected!')
                    print("Exiting.")
                    sys.exit(1)

        print("Flow deck is attached")

        logconf.start() # Start logging
        while True:
            time.sleep(0.0000001)
            # Determine whether the point is white or black
            if shutter_num < COLOUR_THRESHOLD:
                print(f"White. Shutter = {shutter_num}")
            
            else:
                print(f"Black. Shutter = {shutter_num}")

        #logconf.stop() # Stop logging


def log_shutter_callback(timestamp, data, logconf):
    global shutter_num
    shutter_num = data["motion.shutter"]

def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        flow_deck_attached_event.set()


if __name__ == '__main__':
    main()
