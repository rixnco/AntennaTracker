#!/usr/bin/python3

import serial
import sys
import os.path
import getopt
import xml.dom.minidom as minidom
import time
import matplotlib.pyplot as plt


tokill = False


def main(argv):
    # ser = serial.Serial('/dev/tty.-DevB', 115200, timeout=2)
    ser = serial.Serial('COM3', 115200, timeout=2)
    # ser.open()
    # ser.flushInput()
    speed_factor = 1.
    try:
        opts, args = getopt.getopt(argv,"hs:",["sspeed="])
    except getopt.GetoptError:
        print('test.py -s <speed>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-s", "--speed"):
            speed_factor = float(arg)
    inputFile = os.path.abspath(args[0])
    latitudes, longitudes, elevations, times, dTSec = decodeGPX(inputFile)
    dTSec = dTSec / speed_factor
    print("dt : ", dTSec)

    # Send first point for homing the Antenna Tracker
    print("sending homing point")
    # send_gps_fix()
    send_gps_pt_sp(latitudes[0], longitudes[0], 0, ser)
    ser.flushOutput()
    input("Ok, press Enter to continue...")

    # PLOT TRACK
    plt.ion()
    fig = plt.figure(figsize=(8, 4))
    track = fig.add_subplot(111)
    fig.canvas.mpl_connect('key_press_event', press)
    track.set_aspect(1)
    track.set_ylabel("Latitude")
    track.set_xlabel("Longitude")
    track.set_title("Track Plot")
    track.plot(longitudes, latitudes, 'k', longitudes[0], latitudes[0], 'g.')
    # homeplot, = track.plot(longitudes[0], latitudes[0], c='green')
    targetPlot, = track.plot(longitudes[0], latitudes[0], 'r.')
    plt.show()

    t0 = time.process_time()
    for i in range(len(latitudes)):
        while time.process_time() - t0 < dTSec:
            pass
        t0 = time.process_time()
        send_gps_pt_sp(latitudes[i], longitudes[i], elevations[i], ser)
        targetPlot.set_xdata(longitudes[i])
        targetPlot.set_ydata(latitudes[i])
        plt.pause(0.00001)
        if tokill:
            break
    ser.close()


    # Now send points


def decodeGPX(inputFilePath):
    # Open gpx file and parse it
    data = open(inputFilePath)
    xmldoc = minidom.parse(data)
    track = xmldoc.getElementsByTagName('trkpt')
    elevation = xmldoc.getElementsByTagName('ele')
    datetime = xmldoc.getElementsByTagName('time')
    n_track = len(track)
    lon_list = []
    lat_list = []
    h_list = []
    time_list = []
    # pupolate lists
    for s in range(n_track):
        lon, lat = track[s].attributes['lon'].value, track[s].attributes['lat'].value
        elev = elevation[s].firstChild.nodeValue
        lon_list.append(float(lon))
        lat_list.append(float(lat))
        h_list.append(float(elev))
        # PARSING TIME ELEMENT
        dt = datetime[s].firstChild.nodeValue
        time_split = dt.split('T')
        hms_split = time_split[1].split(':')
        time_hour = int(hms_split[0])
        time_minute = int(hms_split[1])
        time_second = float(hms_split[2].split('Z')[0])
        total_second = time_hour * 3600 + time_minute * 60 + time_second
        time_list.append(total_second)

    # detect dT
    dt_avg = (time_list[-1]/n_track)
    return lat_list, lon_list, h_list, h_list, dt_avg

def send_gps_pt_sp(latitude, longitude, elevation, serial_port):
    print("sending ", latitude, longitude)
    message1 = bytearray([0x7E, 0x77, 0x10, 0x00, 0x08])
    latitude_ = 0x00000000
    longitude_ = 0x80000000
    # Send latitude first
    if latitude < 0:
        latitude_ += 0x40000000
    temp = (abs(latitude) * 600000.)
    temp = int(temp)
    temp &= 0x3FFFFFFF
    latitude_ += temp
    latitude_bytes = latitude_.to_bytes(4, 'little', signed=False)
    for b in latitude_bytes:
        message1.append(b & 0xFF)
    message1.append(0x0)
    serial_port.write(message1)
    serial_port.flushOutput()

    message2 = bytearray([0x7E, 0x77, 0x10, 0x00, 0x08])
    # Now send longitude
    if longitude < 0:
        longitude_ += 0x40000000
    temp = (abs(longitude) * 600000.)
    temp = int(temp)
    temp &= 0x3FFFFFFF
    longitude_ += temp
    longitude_bytes = longitude_.to_bytes(4, 'little', signed=False)
    for b in longitude_bytes:
        message2.append(b & 0xFF)
    message2.append(0x0)
    serial_port.write(message2)
    serial_port.flushOutput()

def send_gps_fix():
    global ser
    message = bytearray([0x7E, 0x77, 0x10, 0x04, 0x10])
    data = 0x0
    data += int(600000 * 1010) & 0x3FFFFFFF
    print(data)
    data_bytes = data.to_bytes(4, 'little', signed=False)
    for b in data_bytes:
        message.append(b)
    message.append(0x0)
    for b in message:
        print(hex(b))
    ser.write(message)

def press(event):
    global tokill
    sys.stdout.flush()
    if event.key == 'q':
        tokill = True

if __name__ == "__main__":
   main(sys.argv[1:])

