#!/usr/bin/env python


import cv2
import numpy as np
import os
import rospy
from std_msgs.msg import Float32

qfiles = ['19.png', '66.png']
# qfiles = ['19.png']

if __name__ == "__main__":

    cap = cv2.VideoCapture(0)

    pub = rospy.Publisher('lineDetect', Float32, queue_size=0)
    rospy.init_node('lineDetecter', anonymous=True)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        # the img input
        ret, imgori = cap.read()
        lowbd = np.array([205, 205, 110])
        highbd = np.array([255, 255, 255])
        imgcut = cv2.inRange(imgori, lowbd, highbd)
        ke = np.ones((5, 5))
        imgcut = cv2.erode(imgcut, ke, iterations=3)
        kd = np.ones((10, 10))
        imgcut = cv2.dilate(imgcut, kd)
        imgcut = cv2.Canny(imgcut, 5, 200, 3)

        # imgcut = cv2.dilate(imgcut, np.ones((5, 5)))
        # linePoints = cv2.HoughLinesP(imgcut, 1, np.pi / 180, 80, minLineLength=5, maxLineGap=150)
        # if linePoints is not None:
        #     linePoints = linePoints[:, 0, :]
        #     for x1, y1, x2, y2 in linePoints:
        #         cv2.line(imgori, (x1, y1), (x2, y2), (255, 0, 0), 2)

        lines = cv2.HoughLines(imgcut, 1, np.pi / 180, 90)
        if lines is not None:
            lineInfos = []
            lines1 = lines[:, 0, :]
            for rho, theta in lines1[:]:
                blHasRecorded = False
                for lineInfo in lineInfos:
                    if abs(lineInfo['theta'] - theta) < 0.2:
                        blHasRecorded = True
                        break
                if blHasRecorded:
                    continue
                else:

                    a = np.cos(theta)
                    b = np.sin(theta)

                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * a)
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * a)
                    cv2.line(imgori, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    if y1 == y2:
                        midx = x1
                    else:
                        midx = (x1 - x2) * (240 - y1)/(y1 - y2) + x1
                    lineInfos.append({'theta': theta, 'midx': midx})

            if len(lineInfos) == 1:
                theta = lineInfos[0]['theta']
                midx = lineInfos[0]['midx']
                if abs(theta - np.pi) < 0.5 or abs(theta) < 0.5:
                    if midx > 320:
                        cmdTheta = - np.pi / 4
                    else:
                        cmdTheta = np.pi / 4
                else:
                    cmdTheta = theta * 2

            elif len(lineInfos) == 2:
                thetaSum = lineInfos[0]['theta'] + lineInfos[1]['theta']
                cmdTheta = thetaSum / 2 + np.pi / 2
                if abs(cmdTheta - np.pi) > (np.pi/4 - 0.25):
                    if abs(lineInfos[0]['theta'] - np.pi / 2) < 0.7:
                        midx = lineInfos[1]['midx']
                    else:
                        midx = lineInfos[0]['midx']
                    if midx > 320:
                        cmdTheta = - np.pi * 3 / 4
                    else:
                        cmdTheta = np.pi * 3 / 4
            else:
                print("ERROR!")
                cmdTheta = 0
        else:
            cmdTheta = 0

        if cmdTheta > np.pi:
            cmdTheta = cmdTheta - np.pi * 2

        cv2.line(imgori, (320, 240),
                 (320 + int(np.sin(cmdTheta) * 100), 240 + int(np.cos(cmdTheta) * -100)),
                 (0, 0, 255), 2)

        print cmdTheta / np.pi * 180

        cv2.imshow("fuck", imgori)
        cv2.waitKey(0)

        pub.publish(cmdTheta)
        rate.sleep()
