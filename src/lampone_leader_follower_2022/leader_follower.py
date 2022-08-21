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
    """
        LeaderFollower je hlavni trida pro ulohu Leader -- Follower na Campo Lampone 2022
    """

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

    def wait_for_start(self, method="default"):
        if method == "default":
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

    def detect_leader(self, method="default"):
        detection_center = [0, 0, 0, 0] # (center_y, center_x, size, angle)
        if method == "default":
            img = self.image.copy()
            blob_img_B = (img[:,:,0] < 30)
            blob_img_G = (img[:,:,1] < 30)
            blob_img_R = (img[:,:,2] > 200)
            blob_img = (blob_img_B + blob_img_G + blob_img_R) > 0
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

    def compute_leader_parameters(self, detection, method="default"):
        leader_parameters = [30, 0, 0] # (vzdalenost, relativni_rychlost, uhel -> pouzity k natoceni kol)
        # Zakladni metoda neumi zatacet a rychlost meni nahodne.
        if method == "default":
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

    def compute_control_commands(self, leader_parameters, method="default"):
        left_motor_msg = Int32()
        right_motor_msg = Int32()
        if method == "default":
            if leader_parameters[0] < 10:
                left_motor_msg.data = 0
                right_motor_msg.data = 0
            elif leader_parameters[0] < 20:
                left_motor_msg.data = 20
                right_motor_msg.data = 20
            elif leader_parameters[0] < 30:
                left_motor_msg.data = 50
                right_motor_msg.data = 50
            else:
                left_motor_msg.data = 80
                right_motor_msg.data = 80
            pass
        elif method == "custom":
            # TODO: Implementujte vlastni metodu, ktera parametry leadera prepocita na akcni zasahy pro jednotliva kola. 
            #       Ty ulozte do left_motor_msg a right_motor_msg. 
            # - Tipy: - Kazde kolo ma rozsah -100 az 100. Zaporne hodnoty jsou pro couvani. Zataceni dosahnete rozdilem hodnot na kolech.
            pass
        self.left_motor_pub(left_motor_msg)
        self.right_motor_pub(right_motor_msg)
        pass

    def stop_check(self, method="default"):
        if method == "default":
            pass
        elif method == "custom":
            # TODO: Implementujte metodu pro otestovani konce trate (robot detekuje QR kod prilis blizko po nejakou dobu a v zasade jiz stoji)
            # Ale pozor na začátku bude také nějakou chvíli stát. Nesmí se ukončit před vyjetím.
            pass
        pass

    def emeregency_stop(self):
        # TODO: Implementujte metodu pro nouzove zastaveni
        # Tipy: Může byt pomocí tlacitka nebo pomocí QR kódu. Pokud bude implementováýno pomocí QR kódu tak doporučuji vnořit volání do 
        #       metody detect_leader jelikož tam už se QR kód zpracovává a tím pádem se ušetří výpočetní čas.
        pass

    def run_task(self):
        # TODO: Pockat na signál od uzivatele 
        self.wait_for_start(method="default")  # Po implementaci nastavte parametr na nazev vasi metody.

        # TODO: Iniciovat smycku resici ulohu
        while True:
            detection = self.detect_leader(method="default")
            # TODO: Zpracovat detekci z hlediska vzdalenosti a uhlu
            leader_parameters = self.compute_leader_parameters(detection, method="default")
            # TODO: Udaje pretavit do ovladani robota
            self.compute_control_commands(self, leader_parameters, method="default")
            # TODO: Rozpoznat konec a reagovat zastavenim a vypisem na display.
            self.stop_check(method="default")
            # TODO: Implementovat funkci na nouzove zastaveni.
            self.emergency_stop()


if __name__ == "__main__":
    """
        Vstupni bod aplikace.

        Vytvori node v ROSu a spusti kod ulohy.
    """
    rospy.init_node("leader_follower")
    leader_follower = LeaderFollower()
    leader_follower.run_task()