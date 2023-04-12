#!/usr/bin/env python
'''
  Python script to pass data between X-Plane 11 to STM32PILOT project.
  Handling the messages taken from - https://github.com/charlylima/XPlaneUDP

  Usage: x-plane to stm.py UART_COM UDP_IP UDP_PORT
'''

import sys
from collections import namedtuple
import re
import socket
import struct
import binascii
import serial
import threading
from threading import Event
from datetime import datetime

LOG = False # enable/disable data logging
datarefs = [
    # ( dataref, unit, description, num decimals to display in formatted output )
    ("sim/flightmodel/position/true_theta", "°", "pitch",2),
    ("sim/flightmodel/position/true_phi", "°", "roll",2),
    ("sim/cockpit/pressure/cabin_altitude_actual_m_msl", "ft", "alt",2),
    ("sim/cockpit2/engine/actuators/throttle_ratio_all", "%", "throttle",4),
    ("sim/flightmodel/position/true_psi", "°", "heading",3),
    ("sim/flightmodel/position/latitude", "°", "lat",4),
    ("sim/flightmodel/position/longitude", "°", "lon",4),
    ("sim/flightmodel/position/P","deg/sec","",6),
    ("sim/flightmodel/position/Q","deg/sec","",6),
    ("sim/flightmodel/position/R","deg/sec","",6),
    ("sim/flightmodel/position/true_airspeed","m/sec","",6)
  ]
NUM_DATAREFS = len(datarefs)



def DecodePacket(data):
  retvalues = {}
  # Read the Header "RREFO".
  header=data[0:5]
  if(header!=b"RREF,"):
    print("Unknown packet: ", binascii.hexlify(data))
  else:
    # We get 8 bytes for every dataref sent:
    #    An integer for idx and the float value. 
    values =data[5:]
    lenvalue = 8
    numvalues = int(len(values)/lenvalue)
    idx=0
    value=0
    for i in range(0,numvalues):
      singledata = data[(5+lenvalue*i):(5+lenvalue*(i+1))]
      (idx,value) = struct.unpack("<if", singledata)
      retvalues[datarefs[idx][2]] = (value, datarefs[idx][1], datarefs[idx][2])
  return retvalues

def RequestDataRefs(sock):
  for idx,dataref in enumerate(datarefs):
    # Send one RREF Command for every dataref in the list.
    # Give them an index number and a frequency in Hz.
    # To disable sending you send frequency 0. 
    cmd = b"RREF\x00"
    freq=250
    string = datarefs[idx][0].encode()
    message = struct.pack("<5sii400s", cmd, freq, idx, string)
    assert(len(message)==413)
    sock.sendto(message, (UDP_IP, UDP_PORT))

def WriteDataRef(dataref,value,vtype, sock):
    '''
    Write Dataref to XPlane
    DREF0+(4byte byte value)+dref_path+0+spaces to complete the whole message to 509 bytes
    DREF0+(4byte byte value of 1)+ sim/cockpit/switches/anti_ice_surf_heat_left+0+spaces to complete to 509 bytes
    '''
    cmd = b"DREF\x00"
    dataref  =dataref+'\x00'
    string = dataref.ljust(500).encode()
    message = "".encode()
    if vtype == "float":
      message = struct.pack("<5sf500s", cmd,value,string)
    elif vtype == "int":
      message = struct.pack("<5si500s", cmd, value, string)
    elif vtype == "bool":
      message = struct.pack("<5sI500s", cmd, int(value), string)

    assert(len(message)==509)
    sock.sendto(message, (UDP_IP, UDP_PORT))

def sendCommand(command, data, type):
  try:
    data = round(float(data.decode()),4)
    try:
      WriteDataRef(command, data, 'float', sock)
      #writeLog(command.split("/")[-1], datetime.now().strftime("%H:%M:%S.%f"), str(data))
    except Exception as e:
      print("An error occured while sending {}:".format(command), e)
  except ValueError:
    pass

def writeLog(dataHeader, dataTime, data):
  if LOG:
    logFile.write(dataHeader)
    logFile.write(",")
    logFile.write(dataTime)
    logFile.write(",")
    logFile.write(data)
    logFile.write("\n")

def threadFuncTwo(event):
  ser = serial.Serial(
      port=COM,
      baudrate = 115200,
      parity=serial.PARITY_NONE,
      stopbits=serial.STOPBITS_ONE,
      bytesize=serial.EIGHTBITS
  )
  ack = True

  logs = {"m":[[],[]], "ps":[[],[]], "pc":[[],[]], "rs":[[],[]], "rc":[[],[]], "ts":[[],[]], "tc":[[],[]], "as":[[],[]], "pd":[[],[]], "rd":[[],[]], "ad":[[],[]]} # {"pitch":[[times],[pitch]]}


  while threads_run:
      while ser.in_waiting:
        inputData = ser.readline()
        if inputData == b'SendP\n' and ack == True:
          ser.write(data)
          ack = False
          ser.flush()
        elif inputData == b'Ack\n' and ack == False:
          ack = True
        elif inputData[0:2] == b'PC':
          sendCommand("sim/cockpit2/controls/yoke_pitch_ratio", inputData[2:], 'float')
        elif inputData[0:2] == b'RC':
          sendCommand("sim/cockpit2/controls/yoke_roll_ratio", inputData[2:], 'float')
        elif inputData[0:2] == b'TC':
          sendCommand("sim/cockpit2/engine/actuators/throttle_ratio_all", inputData[2:], 'float')
        elif inputData[0:2] == b'HC':
          sendCommand("sim/cockpit2/controls/yoke_heading_ratio", inputData[2:], 'float')
        elif inputData[0:2] == b'DP':
          sendCommand("sim/cockpit2/controls/yoke_pitch_ratio", inputData[2:], 'float')
        elif inputData[0:2] == b'DR':
          sendCommand("sim/cockpit2/controls/yoke_roll_ratio", inputData[2:], 'float')
        elif inputData[0:2] == b'DT':
          sendCommand("sim/cockpit2/engine/actuators/throttle_ratio_all", inputData[2:], 'float')
        elif inputData[0:2] == b'DH':
          sendCommand("sim/cockpit2/controls/yoke_heading_ratio", inputData[2:], 'float')
        elif inputData[0:5] == b'deb: ':
          print(inputData)
        elif inputData[0:2] == b'P:' or inputData[0:2] == b'I:' or inputData[0:2] == b'D:' or inputData[0:2] == b'X:':
          print(inputData)
        elif inputData[0:4] == b'log:':
          current_time = datetime.now().strftime("%H:%M:%S.%f")
          if inputData[4:5] == b'm':
            writeLog("mode", current_time, str(int(inputData[5:].decode())))
          else:
            writeLog(str(inputData[4:6].decode()), current_time, str(float(inputData[6:].decode())))
  ser.close()


def threadFuncOne():
  # Open a Socket on UDP Port 49000
  global sock
  global data
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
  RequestDataRefs(sock)

  while threads_run:
    try:
      # Listen for incoming packets
      data = b'$' + sock.recv(5+(NUM_DATAREFS*8))
    except socket.error as e:
      print("There was an error while trying to receive data:", e)

def setUp(args):
  arg_names = ['script_path', 'COM', 'UDPIP', 'UDPPORT']
  args = dict(zip(arg_names, args))
  Arg_list = namedtuple('Arg_list', arg_names)
  args = Arg_list(*(args.get(arg, None) for arg in arg_names))
  global UDP_IP, UDP_PORT
  global COM
  global logFile
  if args[1] == None:
    print("No COM port has entered!\nPlease try again with the COM port which connected to the uart adapter")
    raise SystemExit(1)
  else:
    COM = args[1]
    if args[2] != None and args[3] != None:
        isIpGood = re.match(r"[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}", args[2])
        if isIpGood:
          UDP_IP = args[2]
        else:
          print("Error in IP format!\nPlease try again with valid IP. ex. 127.0.0.1")
        isPortGood = re.match(r"^[0-9]+$", args[3])
        if isPortGood:
          UDP_PORT = args[3]
        else:
          print("Error in PORT format!\nPlease try again with valid PORT (only numbers)")
    else:
      # not arguments sent - use default ip port
      UDP_IP = "127.0.0.1"
      UDP_PORT = 49000
    if LOG:
      logFile = open("STM32PILOT_Log-{}.txt".format(datetime.now().strftime("%Y-%m-%d %H-%M")), "w")
    print("Connecting to UART on COM: {}".format(COM))
    print("Connecting to X-Plane via UDP IP: {}, PORT: {}".format(UDP_IP, UDP_PORT))
    logONOFF = 'ON' if LOG else 'OFF'
    print("Data logging is {}".format(logONOFF))

def threadsSetup():
  global threads_run
  global event
  threads_run = True

  th1 = threading.Thread(target=threadFuncOne)
  th1.daemon = True
  th1.start()

  event = Event()
  th2 = threading.Thread(target=threadFuncTwo, args=(event,))
  th2.daemon = True
  th2.start()

  threads = [th1, th2]
  # in order to terminate the script while threads are running, look for CTRL-C command
  try:
      while threads_run:
        [t.join(1) for t in threads if t is not None and t.isAlive()]
  except KeyboardInterrupt:
    threads_run = False
    event.set()
    if LOG:
      logFile.close()
    th1.join()
    th2.join()

if __name__ == '__main__':
  setUp(sys.argv)
  threadsSetup()