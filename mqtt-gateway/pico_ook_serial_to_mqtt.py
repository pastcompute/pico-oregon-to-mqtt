# Dependencies:
# apt-get install python3-serial

import serial
import csv
import re
import time
import math
import sys
import os
import gpiod
from gpiod.line import Direction, Value


# First draft - these need to be from the command line or elsewhere
SERIAL_PORT = "/dev/ttyACM0"
MOSQUITTO_IP = "192.168.1.2"
MOSQUITTO_USER = "USERNAME"
MOSQUITTO_PASS = "PASSWORD"

# Monitor the serial port, look for messages like these, and turn into MQTT
# Lacrosse,281370,24.8,182,-23.7,ok,-53.5,12
# Lacrosse,253106,24.3,143,-17.3,ok,-67.5,11
# STATUS,285782,24.3,-88.8,153,2483,10675
# Oregon,285908,24.3,1d20,1,27,19.9,45,ok,-63.0,6

PIN=23

def buzz(t):
    with gpiod.request_lines(
        "/dev/gpiochip0",
        consumer="buzzer",
        config={
            PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE),
        }
    ) as request:
        request.set_value(PIN, Value.ACTIVE)
        time.sleep(t)
        request.set_value(PIN, Value.INACTIVE)

# TODO - background this so it wont stop other messages
def bips(n, t, w):
    for x in range(n):
        buzz(t)
        if (x+1) < n: time.sleep(w)

# Seconds for serial timeout. Used to unblock and see if we have not had a freezer message since too long
SER_TIMEOUT=15*60

# Complain about missing freezermessages if seen none for longer than this
DEAD_SECONDS=333
# Temperature thresholds
FREEZER_WARM=-11
FRIDGEFREEZ_WARN=-10

print("Starting...")
bips(2,0.1,0.05)
ser = None
port = 0
while True:
    try:
        print("Probing device={}{}".format(SERIAL_PORT_BASE, port))
        serial_port = "{}{}".format(SERIAL_PORT_BASE, port)
        ser = serial.Serial(serial_port, 115200, parity=serial.PARITY_NONE, stopbits=1, rtscts=0, xonxoff=0, timeout=SER_TIMEOUT)
        print("Connected")
        break
    except serial.serialutil.SerialException as es:
        print(es)
        print("Try next port in 5 seconds")
        time.sleep(5)
        port = port + 1
        if (port > 3): port = 0

bips(2,0.1,0.05)

ser.flushInput()

pat = re.compile(" ")

rssi_valid = False
t0 = 0
t1 = 0
nn = 0
tLastFreezer = time.time()
tLastFridge = time.time()
tLastWarn1 = 0
tLastWarn2 = 0


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
    global tLastFreezer
    global tLastFridge
    if int(id) == 182:
        print("deepfreeze")
        channel = 2
        tLastFreezer = time.time()
    elif int(id) == 143:
        print("fridgefreezer")
        tLastFridge = time.time()

    # 182 == Deep Freeze, 143 == Fridge Freezer
    # Hack to beep buzzer when too high
    if int(id) == 182 and float(sensorC) > FREEZER_WARM:
        bips(6,0.2,0.2)
    elif int(id) == 143 and float(sensorC) > FRIDGEFREEZ_WARN:
        bips(5,0.2,0.2)

    # TODO: instead of a single beep, make it start beeping and more often as it gets higher


    # TODO hack if we havent got anything for a long time...

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

        # if no \n at the end then we timed out
        # A bit dodgy, we loose some data if this happens mid message
        # The timeout is very long, we only need it to detect if the freezer has stopped sending updates and there have also been no other messages
        bbb = bytes[-1]
        if bbb != 10:
          print("timeout @ {}".format(t1))

        if tLastFreezer < t1 - DEAD_SECONDS:
          print("No deep-freezer message for more than {} seconds".format(t1 - tLastFreezer), tLastFreezer)
          # dont annoy people! if battery went flat we get a double short beep every 15 minutes
          if tLastWarn1 < t1 - 1800:
            bips(2,0.1,0.15)
            tLastWarn1 = t1
        if tLastFridge < t1 - DEAD_SECONDS:
          print("No fridge-freezer message for more than {} seconds".format(t1 - tLastFridge), tLastFridge)
          if tLastWarn2 < t1 - 1800:
            bips(3,0.1,0.15)
            tLastWarn2 = t1

        if bbb != 10:
          continue

        s = bytes.decode("utf-8").rstrip().lstrip()
        reader = csv.reader([s])
        for row in reader:
            if len(row) > 0 and row[0] == "STATUS":
                print(t1, s)
                uptime = row[2]
                internalTempC = row[3]
                backgroundRssi = row[4]
                break
            if len(row) > 0 and row[0] == "Lacrosse":
                print(t1, s)
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
                print(t1, s)
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
