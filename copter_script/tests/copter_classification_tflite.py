import threading
import time
import sys 
sys.path.append(r'/home/administrator/copter_script')
from multiCopterHandler import *
from cv_bridge import CvBridge
import cv2
import os

altitude = 4
bridge = CvBridge()

class MyCopterHandler(CopterHandler):
    def __init__(self, name):
        super(MyCopterHandler,self).__init__(name)

        self.flight_thread = threading.Timer(0, self.flight_trajectory)
        self.flight_thread.daemon = True
        self.flight_thread.start()

        self.battery_thread = threading.Timer(0, self.battery_condition)
        self.battery_thread.daemon = True
        self.battery_thread.start()

        self.detection_thread = threading.Timer(0, self.objects_detection)
        self.detection_thread.daemon = True
        self.detection_thread.start()

    def flight_trajectory(self):
        time.sleep(1)
        print("arm:", self.armed, "mode:", self.mode)
        if self.connected:
            self.change_mode(MODE_GUIDED)
            self.arm(True)
            self.takeoff(altitude)
            time.sleep(20)
            self.move_local(0,150,altitude) # go from 0,0 to 0,150

    def battery_condition(self):
        time.sleep(1)
        if self.connected:
            print("testing battery")
            while True:
                #print('position: '+str(self.x)+' '+str(self.y)+' '+str(self.z))
                if self.battery<0.2:
                    self.return_to_home()
                    break
            self.disconnect()
    
    def objects_detection(self):
        time.sleep(1)
        t1 = time.time()
        i=0
        if self.connected:
            while True:
                t = time.time()
                if t-t1>self.recognition_rate:
                    t1 = t
                    if self.objects_detected != None:
                        for key in self.objects_detected:
                            print(key)
                            os.chdir('/home/administrator/Pictures/images_detection')
                            cv2.imwrite(key+str(i)+".jpg", self.last_detected_image)
                            i+=1
                            print("photo of "+key+" taken")


if __name__ == "__main__":
    v = MyCopterHandler("drone1")
    v.enable_topics_for_read()
    v.connect("node1", rate=10)