# Dependencies:
# apt-get install python3-serial

import serial
import csv
import re
import time
import math
import sys
import os

SERIAL_PORT_BASE = "/dev/ttyACM"
MOSQUITTO_IP = "192.168.0.2"
MOSQUITTO_USER = "redacted"
MOSQUITTO_PASS = "redacted"


# Monitor the serial port, look for messages like these, and turn into MQTT
# Lacrosse,281370,24.8,182,-23.7,ok,-53.5,12
# Lacrosse,253106,24.3,143,-17.3,ok,-67.5,11
# STATUS,285782,24.3,-88.8,153,2483,10675
# Oregon,285908,24.3,1d20,1,27,19.9,45,ok,-63.0,6

ser = None
port = 0
while True:
    try:
        serial_port = "{}{}".format(SERIAL_PORT_BASE, port)
        ser = serial.Serial(serial_port, 115200, parity=serial.PARITY_NONE, stopbits=1, rtscts=0, xonxoff=0, timeout=None)
        break
    except serial.serialutil.SerialException as es:
        print(es)
        print("Try next port in 5 seconds")
        time.sleep(5)
        port = port + 1
        if (port > 3): port = 0


ser.flushInput()

pat = re.compile(" ")

rssi_valid = False
t0 = 0
t1 = 0
nn = 0


#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/time 2022-06-13 21:25:13
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/id 143
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/channel 1
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/temperature_C -15.2
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/battery_ok 1
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/test No
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/mod ASK
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/freq 433.87741
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/rssi -0.0724373
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/snr 6.67724
#rtl_433/unifi2/devices/LaCrosse-TX141Bv2/1/143/noise -6.74968
# mosquitto_pub -h 172.30.42.19 -u andrew -P andrew -t cmnd/tasmota_972885/POWER -m OFF   
def pubLacrosse(uptime, id, internalC, sensorC, battery, rssi, count):
    RTL_FAKE_DEVICE = "LaCrosse-TX141Bv2"
    # Hack: not parsing channel properly as yet
    channel = 1
    if int(id) == 182:
        channel = 2

    topicBase = f"rtl_433/unifi2/devices/{RTL_FAKE_DEVICE}/{channel}/{id}"

    fields = {"channel":channel, "temperature_C":sensorC, "battery_ok": 1 if battery else 0, "rssi":rssi }
    for k, v in fields.items():
        topic = f"{topicBase}/{k}"
        value = v
        cmd = f"mosquitto_pub -h {MOSQUITTO_IP} -u {MOSQUITTO_USER} -P {MOSQUITTO_PASS} -t {topic} -m {value}"
        #print(id, cmd)
        os.system(cmd)

# rtl_433/unifi2/devices/Oregon-THGR122N/1/111/time 2022-01-25 18:34:17
# rtl_433/unifi2/devices/Oregon-THGR122N/1/111/id 111
# rtl_433/unifi2/devices/Oregon-THGR122N/1/111/channel 1
# rtl_433/unifi2/devices/Oregon-THGR122N/1/111/battery_ok 0
# rtl_433/unifi2/devices/Oregon-THGR122N/1/111/temperature_C 28.7
# rtl_433/unifi2/devices/Oregon-THGR122N/1/111/humidity 74
def pubOregon(uptime, id, internalC, sensorC, sensorHumidity, battery, rssi, count):
    RTL_FAKE_DEVICE = "Oregon-THGR122N"
    topicBase = f"rtl_433/unifi2/devices/{RTL_FAKE_DEVICE}/1/{id}"

    fields = {"channel":1, "temperature_C":sensorC, "humidity": sensorHumidity, "battery_ok": 1 if battery else 0, "rssi":rssi }
    for k, v in fields.items():
        topic = f"{topicBase}/{k}"
        value = v
        cmd = f"mosquitto_pub -h {MOSQUITTO_IP} -u {MOSQUITTO_USER} -P {MOSQUITTO_PASS} -t {topic} -m {value}"
        os.system(cmd)

while True:
    sys.stdout.flush()
    nn = nn + 1
    try:
        bytes = ser.readline()
        t1 = time.time()
        s = bytes.decode("utf-8").rstrip().lstrip()
        reader = csv.reader([s])
        for row in reader:
            if len(row) > 0 and row[0] == "STATUS":
                print(s)
                uptime = row[2]
                internalTempC = row[3]
                backgroundRssi = row[4]
                break
            if len(row) > 0 and row[0] == "Lacrosse":
                print(s)
                uptime = row[1]
                internalTempC = row[2]
                id = row[3]
                sensorTempC = row[4]
                batteryOk = row[5] == "ok"
                rssi = row[6]
                count = row[7]
                pubLacrosse(uptime, id, internalTempC, sensorTempC, batteryOk, rssi, count)
                break
            if len(row) > 0 and row[0] == "Oregon":
                print(s)
                id = 111
                uptime = row[1]
                internalTempC = row[2]
                model = row[3]
                sensorTempC = row[6]
                sensorHumidity = row[7]
                batteryOk = row[8] == "ok"
                rssi = row[9]
                count = row[10]
                pubOregon(uptime, id, internalTempC, sensorTempC, sensorHumidity, batteryOk, rssi, count)
                break
            #print("x", s)
            break
    except KeyboardInterrupt as e:
        break
    except Exception as e2:
        ex = sys.exc_info()[0]
        print(ex)
        print(e2)
        time.sleep(5)
        if re.match(e2.strerror, "device disconnected"):
            # port changed, let systemd restart us
            break
        continue

