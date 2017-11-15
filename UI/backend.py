import socket
import hashlib
import base64
import time
import thread


connectionopen = True
gui = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11" # web socket spec standard key
host = ''
port = 5555


global recieved_data
global thread_close

global s
global s2
global s3

s2 = socket.socket()
host2 = '192.168.1.101'
port2 = 5003
s2.connect((host2,port2))
s2.send("S1")
s2.setblocking(0)

s3 = socket.socket()
host3 = '192.168.1.101'
port3 = 5002
s3.connect((host3,port3))


def pi_recv():
    global s2
    while 1:
        try:
            data = s2.recv(1024)
            print data
            #data_send(data)
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
        sendbytes.append((datalength>>8)&255)
    for value in datatosend: # Add the data to the end of the send bytes array
        sendbytes.append(value)	
    s.send(bytearray(sendbytes))	


def socket_recieve(s):
    global recieved_data
    global thread_close
    data = ""
    while thread_close == False:
        try:
            data = s.recv(4000)
            data = demask(data)
            print data
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
    print ("Waiting For Connection to User Interface")
    s = socket.socket()
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

    print "Connection to UI Made"
    while connectionopen == True:
        time.sleep(0.2)

        try:
            data_send("Hello")
        except:
            thread_close = True
            connectionopen = False
            print "Connection to UI Lost"
        
   