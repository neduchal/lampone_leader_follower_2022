#!/usr/bin/env python3

from pyzbar.pyzbar import decode
import time
import numpy as np
import cv2
import rospy
import random
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

class LeaderFollower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber("/camera",
            Image, callback=self.image_callback, queue_size=1)
        self.display_pub = rospy.Publisher("/display",
            String, queue_size=10)
        self.motors_pub = rospy.Publisher("/cmd_vel",
            Twist, queue_size=10)
        self.left_motor_pub = rospy.Publisher("/cmd_left_motor", Int32, queue_size=10)
        self.right_motor_pub = rospy.Publisher("/cmd_right_motor", Int32, queue_size=10)
        self.image = None
        self.state = "Init Done"
        display_text = String()
        display_text.data = self.state
        self.display_pub.publish(display_text)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def wait_for_start(self, method=None):
        if method is None:
            r = rospy.Rate(1)
            for i in range(60):
                r.sleep()
            return
        elif method == "custom":
            # TODO: Implementujte vlastni metodu pro vyckani na zacatek ulohy. 
            # - Tipy: Je mozne pouzit QR kod, zkusit poslat signal pres ROS  z jineho PC, nebo osadit robota tlacitkem (primo nebo pres arduino)
            pass
        else:
            rospy.logerror("Neznama metoda pro odstartovani ulohy")

    def detect_leader(self, method=None):
        detection_center = (0, 0, 0, 0) # (center_y, center_x, size, angle)
        if method is None:
            img = self.image.copy()
            blob_img = np.logical_and(np.logical_and((img[:,:,0] < 30), (img[:,:,1] < 30)), img[:,:2] > 200)
            detections = np.where(blob_img[:, blob_img.shape[1]//2 - 30 : blob_img.shape[1]//2 + 30 ])
            if len(detections[0]) > 0:
                detection_center = (detections[0][0], blob_img.shape[1]// 2 - 30 + detections[1][0]) 
                return detection_center, random.randint(80, 120), 0
            else:
                return None
        elif method == "custom":
            # TODO: Implementujte vlastni metodu pro detekci leadera. 
            # - Tipy: Je mozne pouzit QR kod, barvu, případně jiný na obraze založený přístup. Podoba Leadera se nesmí měnit.
            pass
        else:
            rospy.logerror("Neznama metoda pro detekci leadera")

    def compute_leader_parameters(self, detection, method=None):
        leader_parameters = (30, 0, 0) # (vzdalenost, relativni_rychlost, uhel -> pouzity k natoceni kol)
        # Zakladni metoda neumi zatacet a rychlost meni nahodne.
        if method is None:
            distance = detection[2] // 4
            leader_parameters[0] = distance
        elif method == "custom":
            # TODO: Implementujte vlastni metodu pro prevod detekce leadera na parametry pouzitelne pro rizeni
            # - Tipy:   - Zmerte si velikost QR kodu (ci jineho poznavaciho znameni robota) v pixelech pri ruznych vzdalenostech followera od leadera. 
            #             Na zaklade ziskanych dat vytvorte funkci, ktera prevede data z detekce na vzdalenost.
            #           - K presnejsimu vypoctu ridicich zasahu muze pomoci znalost rozdilu rychlosti (je nutne vytvorit promennou typu self. )
            #           - Uhel je mozne spocitat z pozice QR kodu ci jineho poznavaciho znameni v obrazu.
            pass
        return leader_parameters

    def compute_control_commands(self, leader_parameters, method=None):
        left_motor_msg = Int32()
        right_motor_msg = Int32()
        if method is None:
            if leader_follower[0] < 10:
                left_motor_msg.data = 0
                right_motor_msg.data = 0
            elif leader_follower[0] < 20:
                left_motor_msg.data = 20
                right_motor_msg.data = 20
            elif leader_follower[0] < 30:
                left_motor_msg.data = 50
                right_motor_msg.data = 50
            else:
                left_motor_msg.data = 80
                right_motor_msg.data = 80
            pass
        elif method == "custom":
            # TODO: Implementujte vlastní metodu, ktera parametry leadera prepocita na akcni zasahy pro jednotliva kola. 
            #       Ty ulozte do left_motor_msg a right_motor_msg. 
            # - Tipy: - Kazde kolo ma rozsah -100 az 100. Zaporne hodnoty jsou pro couvani. Zataceni dosahnete rozdilem hodnot na kolech.
            pass
        self.left_motor_pub(left_motor_msg)
        self.right_motor_pub(right_motor_msg)
        pass

    def stop_check(self, method=None):
        if method is None:
            pass
        elif method == "custom":
            # TODO: Implementujte metodu pro otestovani konce trate (robot detekuje QR kod prilis blizko po nejakou dobu a v zasade jiz stoji)
            pass
        pass

    def run_task(self):
        # TODO: Počkat na signál od uživatele 
        self.wait_for_start(method=None)  # Po implementaci nastavte parametr na nazev vasi metody.

        # TODO: Iniciovat smyčku řešící úlohu
        while True:
            detection = self.detect_leader(method=None)
            # TODO: Zpracovat detekci z hlediska vzdálenosti a úhlu
            leader_parameters = self.compute_leader_parameters(detection, method=None)
            # TODO: Údaje přetavit do ovládání robota
            self.compute_control_commands(self, leader_parameters, method=None)
            # TODO: Rozpoznat konec a reagovat zastavením a výpisem na display.
            self.stop_check(method=None)
            pass

        # TODO: Vymyslet jak evaluovat performance.


if __name__ == "__main__":
    rospy.init_node("leader_follower")
    leader_follower = LeaderFollower()
    leader_follower.run_task()