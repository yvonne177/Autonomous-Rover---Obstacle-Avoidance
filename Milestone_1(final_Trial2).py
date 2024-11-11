'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial

# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.''' 
    start_time = time.time()
    response_raw = ''
    time.sleep(0.21)
    response_raw = SER.read(1024)
    response_new=response_raw.decode("ascii")
   # response_new= ((response_new.split('\',0).split(''')[1]))

    print(f'Raw response was:{response_raw}')
    
    # If response received, return it
    if response_raw:
        return [response_new, datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
##########################TURN OFF###########################
def depacketize(data_raw: str): 
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        print(FRAMESTART + data + FRAMEEND)
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM7'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = False



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0




############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = True # If true, run this. If false, skip it

ct = 0
fix = False
min_dist_corner = 4 #min distance for the corner sensor
min_dist_front = 5
parallel_sensor = 0.2
too_close = 1.5
LOOP_PAUSE_TIME = 0.01
PARALLEL_PAUSE = 0.01
count_turn =0

command=input("Please enter 1 to start:")
#Begin by aligning parallel to walls

if command=='1':
    # Check all ultrasonic sensor 'us'
    packet_tx=packetize('us')
    if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            print(us_cleaned)
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    
    #Compare u1 and u3, check which wall is closest
    #If right wall closer,
    if u1_response > u3_response:
        print("Not Parallel!")
        uneven = True
        
            
        while uneven == True:
            #Checks which side it needs to rotate towards
            if u3_response > u4_response:
                transmit(packetize('r:-10'))
                time.sleep(PARALLEL_PAUSE)
            elif u3_response < u4_response:
                transmit(packetize('r:10'))
                time.sleep(PARALLEL_PAUSE)
            dump=receive()
            
            # Start moving once minimum distance cleared
            packet_tx=packetize('us')
            if packet_tx:
                    transmit(packet_tx)
                    time.sleep(0.3)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic ALL reading: {responses}")
                    us_response=responses[0:len(responses)-2]
                    us_cleaned=us_response.split(',')
                    print(us_cleaned)
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            
            if abs(u3_response-u4_response) <= parallel_sensor:
                uneven = False
                #stops rover once its parallel
                fixed = True
        

clear_serial()

#If left wall closer,
if u1_response < u3_response:
    print("Not Parallel!")
    uneven = True
    while uneven == True:
        
        #Rotates to align with wall
        if u1_response > u2_response:
            transmit(packetize('r:10'))
            time.sleep(PARALLEL_PAUSE)
        elif u1_response < u2_response:
            transmit(packetize('r:-10'))
            time.sleep(PARALLEL_PAUSE)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            print(us_cleaned)
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
            # Start moving once minimum distance cleared
        if abs(u1_response-u2_response) <= parallel_sensor:
            uneven = False
            fixed = True
if u0_float < too_close:
        transmit(packetize('d:-1'))
        time.sleep(0.1)
        dump=receive
        
if u1_response<2:
        #transmit(packetize('d:-1'))
        #time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize('r:-5'))
        time.sleep(LOOP_PAUSE_TIME)
        dump=receive()   
   
if u3_response<2:
        #transmit(packetize('d:-1'))
        #time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize('r:5'))
        time.sleep(LOOP_PAUSE_TIME)
        dump=receive()
clear_serial()   

        



while RUN_COMMUNICATION_CLIENT:
    
   
         
    packet_tx=packetize('us')
    if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            #print(us_cleaned)
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    
    # Check Left Corner
    if u6_response <= min_dist_corner and u0_float > min_dist_front:
        fix = True
        print("Collision!")
        
        #fixes the rotational until fixed
        while fix == True: 
            transmit(packetize('r:-15'))
            print("sent r:-15")
            if packet_tx:
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #   [responses, time_rx] = receive()
            time.sleep(LOOP_PAUSE_TIME)
            #gets u6 value again to check
            packet_tx = packetize('u6')
            if packet_tx:
            
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic 6 reading: {responses}")
                u6_response = float(responses)
            
            # Start moving once minimum distance cleared
            if u6_response > min_dist_corner:
                fix = False   
    
    #Check Right Corner Sensor   
    if u7_response <= min_dist_corner and u0_float > min_dist_front:
        fix = True
        print("Collision!")
        
        #fixes the rotational until fixed
        while fix == True:
    
            packet_tx=(packetize('r:15'))
            print("sent r:15")
            if packet_tx:
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #  [responses, time_rx] = receive() #why do we need this??
            time.sleep(LOOP_PAUSE_TIME)        
            #gets u7 value again to check
            packet_tx = packetize('u7')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 7 reading: {responses}")
                u7_response = float(responses)
                dump=receive()
            
            # Start moving once minimum distance cleared
            if u7_response > min_dist_corner:
                fix = False

    print(f"TRACE",u0_float,u1_response,u2_response,u3_response,u5_response,u6_response,u7_response)   
    time.sleep(LOOP_PAUSE_TIME)
    clear_serial()
    
    #If rover comes up to wall
    if u0_float < min_dist_front: #if its right by the wall
        if u0_float < too_close:
            transmit(packetize('d:-1'))
            time.sleep(0.05)
            dump=receive()
            
        #If right reading is not close to wall and larger than the left reading, turns right
        lanes = [u1_response, u3_response, u5_response]
        if u5_response > 20:
            lanes = [u1_response, u3_response]
        clear_lane = max(lanes)
        
        if clear_lane == u1_response and u2_response>1.2*u0_float:
            transmit(packetize('r:80'))
            time.sleep(LOOP_PAUSE_TIME)
            dump=receive()
        if clear_lane == u3_response and u4_response>1.2*u0_float:
            transmit(packetize('r:-80'))
            time.sleep(LOOP_PAUSE_TIME)
            dump=receive()
        
        if clear_lane == u5_response:
            transmit(packetize('r:-170'))
            time.sleep(1)
            dump=receive()

        
        
    else: #when its far from the wall, checks if theres a better path than going foward
        
    #If right reading is 3x larger than front reading and the front reading is greater than 8, rotate to the right
        if u1_response>1.5*u0_float and u2_response>1.5*u0_float and u0_float>10: #and u0_float<5:
           # transmit(packetize('d:1'))
           # time.sleep(1)
            transmit(packetize('r:80'))
            time.sleep(0.3)
        #If left reading is 3x larger than front reading and the front reading is greater than 8, rotate to the left
        elif u3_response>1.5*u0_float and u4_response>1.5*u0_float and u0_float>10: #and u0_float<5:
            #transmit(packetize('d:1'))
            #time.sleep(1)
            transmit(packetize('r:-80'))
            time.sleep(0.3)
    
    dump=receive()       
    time.sleep(LOOP_PAUSE_TIME)
    transmit(packetize('u0'))
    [responses, time_rx] = receive()  
    time.sleep(LOOP_PAUSE_TIME)
    print(f"Ultrasonic 0 reading: {responses}")
    u0_float=float(responses)
     
        
    #when its far from the wall, checks if theres a better path than going foward
    time.sleep(LOOP_PAUSE_TIME)
    
    print('Forwards!')
    if u0_float > 25:
        transmit(packetize('d:5'))
    
    elif u0_float>min_dist_front:
        transmit(packetize('d:2'))
    
    #Delay to allow rover to travel 3in
    time.sleep(2)
    dump=receive()
  
     
           
        
        #[responses, time_rx] = receive() #dont think we need this
        
