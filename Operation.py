from ultralytics import YOLO
import numpy as np
import cv2
import cvzone
import math
import torch
import socket
import serial
import time
# define constants for socket commnuncation
PORT_NUM = 48952
SEND_DATA_SIZE = 8
SEND_BUFFER_LEN = SEND_DATA_SIZE * 6
REC_DATA_SIZE = 12
REC_DATA_NUM = 7
REC_IO_DATA_SIZE = 3
REC_BUFFER_LEN = REC_DATA_SIZE * 6 + REC_IO_DATA_SIZE + REC_DATA_NUM #

# define a TCP socket/IP
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #AF_INET: Ipv4, SOCK_STREAM: TCP
server_address = (('192.168.1.1', PORT_NUM))

MACHINE_ABS_LINEAR = 1  # MOVE BY ABS COORDINATE VALUE RESPECT MACHINE COORDINATE FRAME USING LINEAR INTERPOLATION
MACHINE_ABS_JOINT = 2  # ...
MACHINE_REALATIVE_LINEAR = 3  # ...
MACHINE_REALATIVE_JOINT = 4  # MOVE BY REALATIVE COORDINATE VALUE RESPECT MACHINE COORDINATE FRAME USING JOINT INTERPOLATION

JOINT_ABS_LINEAR = 5  # MOVE BY ABS COORDINATE VALUE RESPECT JOINT COORDINATE FRAME USING LINEAR INTERPOLATION
JOINT_ABS_JOINT = 6  # ...
JOINT_REALATIVE_LINEAR = 7  # ...
JOINT_REALATIVE_JOINT = 8  # MOVE BY REALATIVE COORDINATE VALUE RESPECT JOINT COORDINATE FRAME USING JOINT INTERPOLATION

OPEN_COMPRESSED_AIR = 9
ClOSE_COMPRESSED_AIR = 10
def socket_initalize():
    # server_address = (('localhost', PORT_NUM))
    print('Connecting to {} port {}'.format(*server_address))
    # Connect the socket to the port where the server is listening
    sock.connect(server_address)
def socket_close():
    sock.close()

def tool_coordinate():
    M = "P"
    M = bytes(M, 'utf-8') # utf-8: Dinh dang chuyen doi Unicode 8 bit
    # M = M.decode('utf-8')
    sock.sendall(M) #gui du lieu thong qua giao thuc TCP
    data = sock.recv(1024) #nhan goi du lieu
    data = data.decode("utf-8") #phan tich goi du lieu vua nhan
    data = data.split(",") # Tach Du lieu = ','
    print("-----------------------")
    print("Current Tool Position")
    print("-----------------------")

    print('X    :  ', data[0])
    print('Y    :  ', data[1])
    print('Z    :  ', data[2])
    print('Roll :  ', data[3])
    print('Pitch:  ', data[4])
    print('Yaw  :  ', data[5])

def joint_coordinate():
    M = "J"
    M = bytes(M, 'utf-8')
    # M = M.decode('utf-8')
    sock.sendall(M)
    data = sock.recv(1024)
    data = data.decode("utf-8")
    data = data.split(",")
    print("-----------------------")
    print("Current Joint Position")
    print("-----------------------")

    print('Joint 1 :  ', data[0])
    print('Joint 2 :  ', data[1])
    print('Joint 3 :  ', data[2])
    print('Joint 4 :  ', data[3])
    print('Joint 5 :  ', data[4])
    print('Joint 6 :  ', data[5])

def move_robot(move_coord, move_mode):
    # wait for machine 'REA'DY status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        sock.sendall(M)
        signal = sock.recv(3)
        if signal == b'REA':
            break
    # data preparation
    x = "{0:8.2f}".format(move_coord[0])
    y = "{0:8.2f}".format(move_coord[1])
    z = "{0:8.2f}".format(move_coord[2])
    r = "{0:8.2f}".format(move_coord[3])
    p = "{0:8.2f}".format(move_coord[4])
    ya = "{0:8.2f}".format(move_coord[5])
    mode = "{:0>3d}".format(move_mode)
    # binding data and converting
    message = x + y + z + r + p + ya + mode
    message = bytes(message, 'utf-8')
    # send
    sock.sendall(message)
    # wait for machine 'FIN'ISH status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        signal = sock.recv(3)
        if signal == b'FIN':
            break

def IO_robot(move_mode):
    # wait for machine 'REA'DY status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        sock.sendall(M)
        signal = sock.recv(3)
        if signal == b'REA':
            break
    # data preparation
    x = "{0:8.2f}".format(0)
    y = "{0:8.2f}".format(0)
    z = "{0:8.2f}".format(0)
    r = "{0:8.2f}".format(0)
    p = "{0:8.2f}".format(0)
    ya = "{0:8.2f}".format(0)
    mode = "{:0>3d}".format(move_mode)
    # binding data and converting
    message = x + y + z + r + p + ya + mode
    message = bytes(message, 'utf-8')
    # send
    sock.sendall(message)
    # wait for machine 'FIN'ISH status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        signal = sock.recv(3)
        if signal == b'FIN':
            break

def write_read(pic):
   data = pic.readline()
   value_ok = data.decode('utf8', 'ignore')
   return value_ok

def currentpoint(pic):
    a = write_read(pic)
    a = a.split("\r")[0]
    a = a.split("\n")[0]
    a = int(a)
    return a

def calib_camera(cem):
    Rc_ext = np.array([[-0.0069, 0.9998, -0.0182], [0.9998, 0.0066, -0.0168], [-0.0167, -0.0183, -0.9997]])
    Tc_ext = np.array([[-232.0157], [-57.7491], [770.4484]])
    Z = Tc_ext[2][0]
    # K = np.array([[668.5852, -0.2297, 320.4217], [0, 667.5386, 238.7476], [0, 0, 1]])  # tool clib
    K = np.array([[667.64, 0, 322.5538], [0, 666.6332, 241.0428], [0, 0, 1]]) # calib thu cong
    mp = np.array([[cem[0]], [cem[1]], [1]])
    T_P_Check = np.linalg.pinv(Rc_ext).dot(np.linalg.pinv(K).dot(Z).dot(mp) - Tc_ext)
    T_P_Check = np.vstack((T_P_Check, 1))
    #print(T_P_Check)
    P1 = np.array([[343.17, -540.2, 105]])  # P1 origin
    P1 = P1.T
    P2 = np.array([[363.14, -539.97,  105]])  # P2 x_direction
    P2 = P2.T
    P3 = np.array([[343.18, -520.11, 105]])  # P3 y_direction
    P3 = P3.T
    # distance |P2 - P1| & |P3 - P1|
    D21 = abs(pow(P2[0][0] - P1[0][0], 2))
    D21 = D21 + abs(pow(P2[1][0] - P1[1][0], 2))
    D21 = D21 + abs(pow(P2[2][0] - P1[2][0], 2))
    D21 = np.sqrt(D21)
    D31 = abs(pow(P3[0][0] - P1[0][0], 2))
    D31 = D31 + abs(pow(P3[1][0] - P1[1][0], 2))
    D31 = D31 + abs(pow(P3[2][0] - P1[2][0], 2))
    D31 = np.sqrt(D31)
    # print('Distance = ',D31) # test distance
    a = (P2[0][0] - P1[0][0]) / D21
    b = (P2[1][0] - P1[1][0]) / D21
    c = (P2[2][0] - P1[2][0]) / D21
    X = np.array([[a], [b], [c]])

    a = (P3[0][0] - P1[0][0]) / D31
    b = (P3[1][0] - P1[1][0]) / D31
    c = (P3[2][0] - P1[2][0]) / D31
    Y = np.array([[a], [b], [c]])

    a = X[1, 0] * Y[2, 0] - X[2, 0] * Y[1, 0]
    b = -X[0, 0] * Y[2, 0] + X[2, 0] * Y[0, 0]
    c = X[0, 0] * Y[1, 0] - X[1, 0] * Y[0, 0]
    Z = np.array([[a], [b], [c]])

    T_Checker_robot = np.hstack((X, Y, Z, P1))
    TP_robot = T_Checker_robot.dot(T_P_Check)
    return TP_robot

home_pos = np.array([400, -20, 300, 90, 0, 0])
position1 = np.array([20, -580, 240, 90, 0, 0])
position2 = np.array([20, -325, 240, 90, 0, 0])
if __name__ == "__main__":
    socket_initalize()
    joint_coordinate()
    tool_coordinate()
    print("camera running")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open webcam")
        exit()
    cap.set(3, 640)
    cap.set(4, 480)
    #pic = serial.Serial(port='COM4', baudrate=9600)
    print("move to home position")
    move_robot(home_pos, 1)
    #load model
    model = YOLO('bestv8l_update.pt')
    className = ['Dirty Bottle','Bottle']
    while True:
        success, img = cap.read()
        if not success:
            print("could not read frame")
            exit()
        img[0:120, 0:640] = [255, 255, 255]
        img[350:480, 0:640] = [255, 255, 255]
        img[0:480, 0:10] = [255, 255, 255]

        results = model(img, stream=True)
        max_conf = 0
        center = np.array([0.0,0.0])
        for r in results:
            boxes = r.boxes
            for box in boxes:
                #Bounding Box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w,h = x2-x1, y2-y1
                Pointx = x1 + w/2
                Pointy = y1 + h/2
                #cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                cvzone.cornerRect(img,(x1,y1,w,h))
                # confidence
                conf = math.ceil((box.conf[0]*100))/100
                #class name
                cls = int(box.cls[0])
                cvzone.putTextRect(img, f'{className[cls]} {conf}', (max(0, x1), max(0, y1 - 20)),scale = 1, thickness = 1)
                if (max_conf < conf):
                    max_conf = conf
                    center = np.array([Pointx,Pointy])
                    classes = cls
        cv2.imshow("Image", img)
        if((max_conf >= 0.9) & (center[1] >140) & (center[1]<300) & (center[0]>=150)):
            cv2.imshow("Image", img)
            #sp = currentpoint(pic)
            sp = time.time()
            print(center)
            TP_robot = calib_camera(center)
            print(TP_robot)
            x = round(TP_robot[0][0], 2)
            y = round(TP_robot[1][0], 2)
            z = round(TP_robot[2][0], 2)
            d = [x, y, z]
            position_near_obj_1 = np.array([x, y + 50, 240, 90, 0, 0])
            print("move to position near the object 1")
            move_robot(position_near_obj_1, 1)
            position_near_obj_2 = np.array([x, y + 50, 140, 90, 0, 0])
            print("move to position near the object 2")
            move_robot(position_near_obj_2, 2)
            while True:
                crp = time.time()
                t = crp - sp
                '''if encc < 0:
                    sp = 0'''
                print(t)
                if t >= 2:
                    position_Obj = np.array([x, y + 50, 100, 90, 0, 0])
                    print("move to object's position")
                    move_robot(position_Obj, 3)
                    print("open compress air to hold the object")
                    IO_robot(10)
                    time.sleep(0.3)
                    position4 = np.array([x, y + 50, 240, 90, 0, 0])
                    print("move to position4")
                    move_robot(position4, 2)
                    if classes == 0:
                        print("move to position of Dirty Bottle")
                        move_robot(position1, 1)
                    else:
                        print("move to position of Bottle")
                        move_robot(position2, 1)
                    print("close compress air to drop the object")
                    IO_robot(9)
                    break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            #pic.close()
            break
    cap.release()
    cv2.destroyAllWindows()