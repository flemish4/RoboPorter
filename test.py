import numpy
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


def recieve_map(filename):
    query = "SELECT data FROM `%u` WHERE type = 'image'" %filename
    cursor.execute(query)
    result = cursor.fetchone() ; 
    img_string = base64.b64decode(result[0])
    nparray = numpy.fromstring(img_string,numpy.uint8) ; 
    image = cv2.imdecode(nparray, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    return image

data = recieve_map(3)

cv2.circle(data,(100,150),100,(255,255,255),50)

send_map(data) 
cursor.close()
connection.close()