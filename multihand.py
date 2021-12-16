# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2
import numpy
from handtracking import HandDetector
from tkinter import font
import paho.mqtt.client as mqtt
from random import randrange, uniform
import time

cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8, maxHands=2)
point = (0, 0)
distFromPoint = ""
mqttBroker = "192.168.12.106"

client = mqtt.Client("nodemcu1")
client.connect(mqttBroker)


def click_event(event, x, y, flags, params):
    global point
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window

        point = (x, y)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window

        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]

        point = (x, y)


while True:

    success, img = cap.read()
    hands, img = detector.findHands(img)  # With Draw
    # hands = detector.findHands(img, draw=False)  # No Draw

    if hands:
        # Hand 1
        hand1 = hands[0]
        lmList1 = hand1["lmList"]  # List of 21 Landmarks points
        bbox1 = hand1["bbox"]  # Bounding Box info x,y,w,h
        centerPoint1 = hand1["center"]  # center of the hand cx,cy

        handType1 = hand1["type"]  # Hand Type Left or Right
        # index1 = hand1["index"]

        # print(index1)
        # print(len(lmList1),lmList1)
        # print(bbox1)
        # print(centerPoint1)
        fingers1 = detector.fingersUp(hand1)
        # length, info, img = detector.findDistance(lmList1[8], lmList1[12], img) # with draw
        # length, info = detector.findDistance(lmList1[8], lmList1[12])  # no draw
        distance, info, img = detector.findDistance(point, centerPoint1, img)  # with draw
        # cv2.putText(img, " distance from hand {} ".format(distance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
        #  0.5, (0, 255, 255), 2, cv2.LINE_4)
        onehandCoordinates = numpy.subtract(centerPoint1, point)
        if onehandCoordinates[0] >= 20:
            x1 = 2
        if -20 <= onehandCoordinates[0] <= 20:
            x1 = 1
        if -20 >= onehandCoordinates[0]:
            x1 = 0
        if onehandCoordinates[1] >= 20:
            y1 = 2
        if -20 <= onehandCoordinates[1] <= 20:
            y1 = 1
        if -20 >= onehandCoordinates[1]:
            y1 = 0
        client.publish("Coordinates", "1{}{}00".format(x1, y1))


        if len(hands) == 2:

            hand2 = hands[1]
            lmList2 = hand2["lmList"]  # List of 21 Landmarks points
            bbox2 = hand2["bbox"]  # Bounding Box info x,y,w,h
            centerPoint2 = hand2["center"]  # center of the hand cx,cy
            handType2 = hand2["type"]  # Hand Type Left or Right

            fingers2 = detector.fingersUp(hand2)
            # print(fingers1, fingers2)
            # length, info, img = detector.findDistance(lmList1[8], lmList2[8], img) # with draw
            length, info, img = detector.findDistance(centerPoint1, centerPoint2, img)  # with draw
            cv2.putText(img, " hands at {} {}".format(centerPoint1, centerPoint2), (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2, cv2.LINE_4)
            distance, info, img = detector.findDistance(point, centerPoint2, img)  # with draw
            distance2, info, img = detector.findDistance(point, centerPoint1, img)  # with draw

            cv2.putText(img, " distance from hand1 {} ".format(distance), (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2, cv2.LINE_4)
            cv2.putText(img, " distance from hand2 {} ".format(distance2), (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2, cv2.LINE_4)
            hand1Coordinates = numpy.subtract(centerPoint1, point)
            hand2Coordinates = numpy.subtract(centerPoint2, point)

            distFromPoint = "{}, {},{},{}".format(hand1Coordinates[0], hand1Coordinates[1], hand2Coordinates[0],
                                                  hand2Coordinates[1])
            if  hand1Coordinates[0] >= 20:
                x1 = 2
            if hand2Coordinates[0] >= 20:
                x2 = 2
            if hand1Coordinates[1] >= 20:
                y1 = 2
            if hand2Coordinates[1] >= 20:
                y2 = 2
            if -20 <= hand1Coordinates[0] <= 20:
                x1 = 1
            if -20 >= hand1Coordinates[0]:
                x1 = 0
            if -20 <= hand2Coordinates[0] <= 20:
                x2 = 1
            if -20 >= hand2Coordinates[0]:
                x2 = 0
            if -20 <= hand1Coordinates[1] <= 20:
                y1 = 1
            if -20 >= hand1Coordinates[0]:
                y1 = 0
            if -20 <= hand2Coordinates[1] <= 20:
                y2 = 1
            if -20 >= hand2Coordinates[1]:
                y2 = 0
            cv2.putText(img, distFromPoint, (10, 100), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2, cv2.LINE_4)
            client.publish("Coordinates","0{}{}{}{}".format(x1,y1,x2,y2))
        # print("Just published " + str(distFromPoint) + " to topic TEMPERATURE")

    cv2.setMouseCallback('Image', click_event)
    cv2.putText(img, str(point), point, cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 0, 0), 2)

    cv2.imshow("Image", img)

    cv2.waitKey(1)
