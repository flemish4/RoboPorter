import socket
import hashlib
import base64
import time
import thread
import logging

import logging.handlers
#the mask for data logging
logging.basicConfig(format='%(asctime)s - (%(threadName)s) %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                    level=logging.INFO) #change the logging level here to change the display verbosity


connectionopen = True
gui = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11" # web socket spec standard key
host = ''
port = 5555


global recieved_data
global thread_close

# s  Connection from backend to UI
# s2 Debug info
# s3 Send commands to Control PI


global s
global s2
global s3
connected2 = False
connected3 = False

s2 = socket.socket()
host2 = '192.168.0.2'
port2 = 5003
logging.info("Connecting to Socket 2")
while connected2 == False:
    connected2 = True 
    try:
        s2.connect((host2,port2))
    except:
        connected2 = False
    time.sleep(0.2)
logging.info("Connected to Socket 2")
s2.send("S1")
s2.setblocking(0)


s3 = socket.socket()
host3 = '192.168.0.2'
port3 = 5002

logging.info("Connecting to Socket 3")
while connected3 == False:
    connected3 = True 
    try:
        s3.connect((host3,port3))
    except:
        connected3 = False
    time.sleep(0.2)
logging.info("Connected to Socket 3")
s3.send("Hello")

def pi_recv(): # Data from Control PI
    global s2
    logging.info("Started Debug Recieve Thread")
    while 1:
        try:
            data = s2.recv(1024)
            # print data
            data_send(data)
        except:
            pass


def pi_send(data):
    global s3
    s3.send(data)
    

def demask(data):
    databytes = []
    xordatabytes = []
    finalstring = ""
    maskindex = 0 
    for eachvalue in data:   
        databytes.append(ord(eachvalue))
    firstmaskbyte = 2
    if databytes[0] != 138 and databytes[0] !=136 : #If not a pong or a closing frame. See data framing in websocket protocol for more info
        mask = [databytes[firstmaskbyte] , databytes[firstmaskbyte+1] , databytes[firstmaskbyte+2] , databytes[firstmaskbyte+3]]
        del databytes[0:6]
        for value in databytes:
            xordatabytes.append(value^mask[maskindex]) # XOR real data and mask bytes to find actual server data
            if maskindex == 3: # Check if the mask index is 3 and if so set it back to 0 as there are only 4 mask bytes
                maskindex = 0
            else:
                maskindex += 1

        for value in xordatabytes: #convert decimal list to string
            finalstring+=chr(value) # append each value to string
        return finalstring;

def ping(): #try to ping UI to check connection
    global s
    sendbytes = []
    sendbytes.append(137)
    sendbytes.append(0)
    s.send(bytearray(sendbytes))

def data_send(datatosend):
    global s
    datalength = len(datatosend) # Gets length
    sendbytes = []
    sendbytes.append(129) # Sets the first byte to 129		
    if datalength <= 125:
        sendbytes.append(datalength) # If length less than 126 set the second send byte to be equal to the length of the data
    elif datalength >=126 and datalength <= 65535: # If greater than 126 append 126 then append bit shifted versions of the data. See here for more info: http://stackoverflow.com/questions/8125507/how-can-i-send-and-receive-websocket-messages-on-the-server-side
        sendbytes.append(126)
        sendbytes.append((datalength>>8)&255)
        sendbytes.append((datalength)&255)
    for value in datatosend: # Add the data to the end of the send bytes array
        sendbytes.append(value)	
    s.send(bytearray(sendbytes))	


def socket_recieve(s): # Data from User Interface
    global recieved_data
    global thread_close
    data = ""
    while thread_close == False:
        try:
            data = s.recv(4000)
            data = demask(data)
            logging.info("Command from interface recieved ="+data)
            pi_send(data)
        except:
            pass

thread.start_new_thread(pi_recv,())

#MAIN LOOP BELOW
while 1:
    thread_close = False
    databytes = []
    xordatabytes = []
    finalstring = ""
    logging.info("Waiting For Connection to User Interface")
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    s.bind((host,port))
    s.listen(0)
    s.setblocking(1)
    s, addr = s.accept()
    rawdata = s.recv(4000)# blocking socket waits for data to be recieved
    data = rawdata.decode('utf-8')
    individual = data.splitlines() # split data into individual lines and put into list
    for line in individual:
        splitline = line.partition(':')
        if splitline[0] == "Sec-WebSocket-Key": 
            key = splitline[2].strip() 

    jointkey = key+gui
    jointkey = hashlib.sha1(jointkey) # hash with sha1
    jointkey = base64.b64encode(jointkey.digest())
    handshake = '''
HTTP/1.1 101 Web Socket Protocol Handshake\r
Upgrade: WebSocket\r
Connection: Upgrade\r
WebSocket-Origin: http://localhost:80\r
WebSocket-Location: ws://localhost:5555/\r
WebSocket-Protocol: sample
Sec-Websocket-Accept:'''+jointkey+''' 
 '''
    handshake = handshake.strip() + '\r\n\r\n'
    s.send(handshake)
    s.setblocking(0)
    connectionopen = True

    thread.start_new_thread(socket_recieve,(s,))

    logging.info("Connection to UI Made")
    while connectionopen == True:
        time.sleep(0.2)
        try:
            ping()
        except:
            thread_close = True
            connectionopen = False
            logging.info("Connection to UI Lost")
        
   