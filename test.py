import numpy as np
import cv2
import MySQLdb 
import base64

global connection
global cursor

connection = MySQLdb.connect(host='192.168.0.1',db='roboporter',user='admin',passwd='password', port=3306)
cursor = connection.cursor()

def send_map(map_to_send):
    img_string = cv2.imencode('.jpg',map_to_send)[1].tostring()
    img_string = base64.b64encode(img_string)
    query = "UPDATE temp SET data = '%s' WHERE type = 'image'" %img_string
    cursor.execute(query)


def recieve_map(name):
    img_string = cv2.imencode('.jpg',map_to_send)[1].tostring()
    img_string = base64.b64encode(img_string)
    print len(img_string)
    query = "UPDATE temp SET data = '%s' WHERE type = 'image'" %img_string
    cursor.execute(query)


data = np.zeros([500,500,3],np.uint8)

cv2.circle(data,(360,456),100,(255,255,255),50)

send_map(data) 

cursor.close()
connection.close()