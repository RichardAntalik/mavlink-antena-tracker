#
# Comm stuff
# This assumes arduino driving motors, servos, steam engines, pistons or whatever.
#

import serial
import struct
import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    print("{}: {} [{}]".format(port, desc, hwid))
    break
ser = serial.Serial(port, 9600)

#
# Geometry stuff
#

from geographiclib.geodesic import Geodesic
import math

def get_bearing(lat1, long1, lat2, long2):
    return Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']

def get_distance(lat1, long1, lat2, long2):
    return Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['s12']

# Shut up, earth is a doughnut... A hollow doughnut.
def get_elevation_angle(distance, height):
    if distance == 0 or height == 0:
        return 0
    diagonal = math.sqrt(distance ** 2 + height ** 2)
    return math.degrees(math.acos((distance ** 2 + diagonal ** 2 - height ** 2) / (2 * diagonal * distance)))

#
# Mavlink stuff
#

import time
from pymavlink import mavutil

# This sucks... there is no docs on string format, no descriptive exceptions, nothing...
master = mavutil.mavlink_connection('tcp:192.168.1.1:14550')

# Local proxy, for debugging
# master = mavutil.mavlink_connection('udpin:localhost:6666')

# Make sure the connection is valid
master.wait_heartbeat()

home_lon = 0
home_lat = 0
home_alt = 0 # always will be, relative alt is available.

vehicle_lon = 0
vehicle_lat = 0
vehicle_alt = 0

home_pos_valid = False
vehicle_pos_valid = False

bearing = 0
elevation_angle = 0

while True:
    #print("getting msg")
    try:
        message = master.recv_msg().to_dict()
        #print("got msg")
    except:
        #print("no  msg")
        message = None
        
    if message and message['mavpackettype'] == 'HOME_POSITION':
        home_lon = int(message['longitude']) / 10000000
        home_lat = int(message['latitude']) / 10000000
        if home_lon != 0 and home_lat != 0:
            home_pos_valid = True
        # print('Home - lon:', home_lon, ' lat:', home_lat)
    if message and message['mavpackettype'] == 'GLOBAL_POSITION_INT':
        vehicle_lon = int(message['lon']) / 10000000
        vehicle_lat = int(message['lat']) / 10000000
        vehicle_alt = int(message['relative_alt']) / 1000
        if vehicle_lon != 0 and vehicle_lat != 0:
            vehicle_pos_valid = True
        # print('Vehicle - lon:', vehicle_lon, ' lat:', vehicle_lat, ' alt:', vehicle_alt)

        if home_pos_valid and vehicle_pos_valid:
            bearing = get_bearing(home_lat, home_lon, vehicle_lat, vehicle_lon)
            distance = get_distance(home_lat, home_lon, vehicle_lat, vehicle_lon)
            elevation_angle = get_elevation_angle(distance, vehicle_alt)
            print('Bearing: ', bearing, ' elevation angle: ', elevation_angle, ' distance: ', distance)
            ser.write(struct.pack("<ff", bearing, elevation_angle))
            #print("serial comm done")
            #while True:
            #    print(ser.readline())

    
    # Somehow sleep causes stream to lag extremely. even small sleep times do this... dunno why...
    #time.sleep(0.0001)
    