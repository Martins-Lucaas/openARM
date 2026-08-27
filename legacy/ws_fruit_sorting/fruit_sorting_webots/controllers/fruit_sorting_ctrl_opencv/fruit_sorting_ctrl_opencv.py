import cv2
import numpy as np
from controller import Supervisor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
ros_node = rclpy.create_node('fruit_sorting_state_publisher')
state_pub = ros_node.create_publisher(String, 'fruit_sorting/state', 10)

last_published_state = None 

def publish_state(state_code):
    global last_published_state
    if state_code == last_published_state:
        return 

    state_names = {
        0: "WAITING",
        1: "PICKING",
        2: "ROTATING",
        3: "DROPPING",
        4: "ROTATE_BACK",
        5: "DISCARD_MOVE",
        6: "DISCARD_RELEASE",
        7: "DISCARD_RETURN"
    }

    msg = String()
    msg.data = state_names.get(state_code, "UNKNOWN")
    state_pub.publish(msg)
    last_published_state = state_code

robot = Supervisor()
timestep = 32  # ms

fruit = -1
orange = 0
apple = 0
discard = 0
counter = 0
state = 0

target_positions = [-1.570796, -1.87972, -2.139774, -2.363176, -1.50971]
discard_target = [-1.570796, -1.87972, 2.50, -2.363176, -1.50971]
speed = 2.0

hand_motors = [
    robot.getDevice('finger_1_joint_1'),
    robot.getDevice('finger_2_joint_1'),
    robot.getDevice('finger_middle_joint_1')
]

ur_motors = [
    robot.getDevice('shoulder_pan_joint'),
    robot.getDevice('shoulder_lift_joint'),
    robot.getDevice('elbow_joint'),
    robot.getDevice('wrist_1_joint'),
    robot.getDevice('wrist_2_joint')
]

for m in ur_motors:
    m.setVelocity(speed)

distance_sensor = robot.getDevice('distance sensor')
distance_sensor.enable(timestep)
position_sensor = robot.getDevice('wrist_1_joint_sensor')
position_sensor.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

def resetDisplay():
    display.setAlpha(0.0)
    display.fillRectangle(0, 0, 200, 150)
    display.setAlpha(1.0)

def printDisplay(x, y, w, h, name):
    resetDisplay()
    display.drawRectangle(x, y, w, h)
    display.drawText(name, x - 2, y - 20)

def findFruit():
    hsv_ranges = [
        (np.array([10, 135, 135], np.uint8), np.array([32, 255, 255], np.uint8)),  # Laranja
        (np.array([30,  50,  50], np.uint8), np.array([90, 255, 255], np.uint8))   # Apple (verde)
    ]
    names = ['Laranja', 'Verde']
    img = np.frombuffer(camera.getImage(), dtype=np.uint8)
    img = img.reshape((camera.getHeight(), camera.getWidth(), 4))
    roi = img[0:150, 35:165]
    imHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    model = -1

    for i, (mn, mx) in enumerate(hsv_ranges):
        mask = cv2.inRange(imHSV, mn, mx)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        cnts = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if w > 80:
                model = i
                printDisplay(x + 35, y, w, h, names[i])
    return model

while robot.step(timestep) != -1:
    publish_state(state)  # publica só se mudar

    if counter <= 0:
        if state == 0:
            if abs(position_sensor.getValue()) > 0.05:
                for m in ur_motors:
                    m.setPosition(0.0)
                for m in hand_motors:
                    m.setPosition(m.getMinPosition())
            else:
                fruit = findFruit()
                if distance_sensor.getValue() < 500:
                    if fruit >= 0:
                        state = 1
                        if fruit == 0:
                            orange += 1
                        elif fruit == 1:
                            apple += 1
                        counter = 8
                        for m in hand_motors:
                            m.setPosition(0.52)
                    else:
                        state = 5
                        counter = 8
                        printDisplay(50, 50, 100, 50, 'Descarte')
                        for m in hand_motors:
                            m.setPosition(0.52)

        elif state == 1:  # PICKING
            for i in range(fruit, 5):
                ur_motors[i].setPosition(target_positions[i])
            state = 2

        elif state == 2:  # ROTATING
            if position_sensor.getValue() < -2.3:
                counter = 8
                state = 3
                resetDisplay()
                for m in hand_motors:
                    m.setPosition(m.getMinPosition())

        elif state == 3:  # DROPPING
            for i in range(fruit, 5):
                ur_motors[i].setPosition(0.0)
            state = 4

        elif state == 4:  # ROTATE_BACK
            if position_sensor.getValue() > -0.1:
                state = 0

        elif state == 5:  # DISCARD_MOVE
            for i in range(5):
                ur_motors[i].setPosition(discard_target[i])
            state = 6

        elif state == 6:  # DISCARD_RELEASE
            if position_sensor.getValue() < discard_target[3] + 0.1:
                for m in hand_motors:
                    m.setPosition(m.getMinPosition())
                discard += 1
                state = 7

        elif state == 7:  # DISCARD_RETURN
            for m in ur_motors:
                m.setPosition(0.0)
            if abs(position_sensor.getValue()) < 0.05:
                state = 0
    else:
        counter -= 1

    label = f'Verde: {apple:3d}    Laranja: {orange:3d}    Descarte: {discard:3d}'
    robot.setLabel(1, label, 0.3, 0.96, 0.06, 0x000000, 0, 'Lucida Console')
rclpy.shutdown()
