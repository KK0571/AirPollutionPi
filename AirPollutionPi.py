import time as t
import os
import glob
from bluetooth import *
import serial.tools.list_ports
import serial
import datetime
import os
import threading

def getSerialPort():
    """
    Automatically detect which serial port the gas sensor is connected to
    """
    ports = list(serial.tools.list_ports.comports())

    for port_no, description, address in ports:
        if 'UART' in description:
            return port_no  # if UART is in description, correct port
    return None

def parseOutput(output):
    """
    Given a serial output, parse the result and extract the fields
    """
    # output always has the same format where fields are separated with ','
    components = output.split(',')
    # check that the output is of the correct length, should be 11 fields
    if len(components) == 11:
        serialNr   = components[0].strip()  # use strip to get rid of whitespaces
        value      = components[1].strip()
        temp       = components[2].strip()
        humid      = components[3].strip()
        gasADC     = components[4].strip()
        tempADC    = components[5].strip()
        humidADC   = components[6].strip()
        days       = components[7].strip()
        hours      = components[8].strip()
        minutes    = components[9].strip()
        seconds    = components[10][:3].strip()
        # return a tuple with all parsed fields
        return (serialNr, value, temp, humid, gasADC, tempADC, humidADC, days, hours, minutes, seconds)
    else:
        return None

def readSensor():
    """
    Trigger a read event on the serial port
    """
    cmd = '\r\n' # command to trigger an output on the serial port
    ser.write(cmd.encode('ascii'))  # write command to the serial port
    s = ser.readline()              # capture the result
    return parseOutput(s.decode())  # decode the result to obtain string

def getSensorType():
    """
    Automatically detect which sensor is connected
    """
    if ser.isOpen():
        ser.close()
        ser.open()
    coldStart = '\n'
    ser.write(coldStart.encode('ascii'))
    t.sleep(1)

    cmd = 'e'   # 'e' triggers EEPROM readout which includes gas type
    ser.write(cmd.encode('ascii'))
    print "wrote command to serial port"
    res = None
    while True:
        s = ser.readline()
        # the line we are interested in
        if 'Gas' in s:
            res = s.decode().split("=")[1].strip() # extract sensor type
            # read rest of lines in order to prevent unwanted printouts
            while 'Sensitivity_Code' not in s:
                s = ser.readline()
            print "done with command"
            return res

def awaitConnection():
    """
    Blocking function which waits for the bluetooth device to initiate connection
    """
    global client_sock, client_info, s_thread, r_thread
    print "Waiting for connection on RFCOMM channel %d" % port

    client_sock, client_info = server_sock.accept()
    print "Accepted connection from ", client_info


def sendThread():
    """
    The sending thread responsible for sending data to the bluetooth device
    """
    while True:
        try:
            res = readSensor()
            if res != None:
                (serialNr, value, temp, humid, gasADC, tempADC, humidADC, days, hours, minutes, seconds) = res
                #print("Reading sensor; Value: " + str(value) + ", Temp: " + str(temp))
                time = datetime.datetime.now()
                data = SENSOR + "," + value + "," + temp + "," + str(time) + "," + serialNr + '!'
                client_sock.send(data)
                print "sending [%s]" % data
            else:
                print("Sensor did not return correct output, retrying")
                continue
            t.sleep(1)
        except btcommon.BluetoothError:
            print "Connection to device lost, waiting for new connection.."
            awaitConnection()   # blocking call

def receiveThread():
    """
    The receiving thread responsible for listening for commands from the bluetooth device
    """
    while True:
        try:
            data = client_sock.recv(1024)
            if data == 'shutdown':
                print "received [%s]" % data
                if data == 'shutdown':
                    client_sock.close()
                    os.system("sudo shutdown -h now")
                    break
        except btcommon.BluetoothError:
            continue

client_sock = None
client_info = None

s_thread = None
r_thread = None

# serial port setup for gas sensor
PORT = getSerialPort()
if PORT == None:
    print("Gas sensor was not detected, aborting!")
    sys.exit(1)

# initialize serial port
ser = serial.Serial(
    port     = PORT,
    baudrate = 9600,
    parity   = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)

t.sleep(1)  # wait for serial to be ready

# check if serial port was successfully opened
if ser.isOpen():
    print("Sensor connected and detected..")

SENSOR = getSensorType()
if SENSOR == None:
    print("Could not determine sensor type, exiting..")
    sys.exit(1)

print("Connected sensor: %s" % SENSOR)

server_sock=BluetoothSocket( RFCOMM ) # create bt socket using RFCOMM protocol
server_sock.bind(("",PORT_ANY))       # bind socket to bt adapter of device on any port
server_sock.listen(1)                 # allow one connection at a time

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"   # same uuid here and on the other device

advertise_service( server_sock, "ASTMOS",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ]
                 )

awaitConnection()   # blocking call

# start threads which handles incoming and outgoing bt communication
s_thread = threading.Thread(target=sendThread)
s_thread.start()
r_thread = threading.Thread(target=receiveThread)
r_thread.start()
