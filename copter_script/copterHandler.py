import threading
import time

import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg
import geographic_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import rospy


from rospyHandler import RosHandler
from topicService import TopicService

MODE_INITIALISING = "INITIALISING"
MODE_MANUAL = "MANUAL"
MODE_STABILIZE = "STABILIZE"
MODE_GUIDED = "GUIDED"
MODE_LAND = "LAND"


class CopterHandler(RosHandler):
    def __init__(self):
        super(CopterHandler,self).__init__()
        self.armed = False
        self.mode = ""
        self.battery = 0
        self.returning_to_home = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.home_x = 0
        self.home_y = 0
        self.home_z = 0


        self.TOPIC_STATE = TopicService("/mavros/state", mavros_msgs.msg.State)
        self.TOPIC_SET_POSE_GLOBAL = TopicService("/mavros/setpoint_position/global", geographic_msgs.msg.GeoPoseStamped)
        self.TOPIC_SET_POSE_LOCAL = TopicService("/mavros/setpoint_position/local", geometry_msgs.msg.PoseStamped)
        self.TOPIC_BATTERY_STATE = TopicService("/mavros/battery", sensor_msgs.msg.BatteryState)
        self.TOPIC_GET_LOCAL_POSE = TopicService("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped)

        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.SERVICE_TAKEOFF = TopicService("/mavros/cmd/takeoff", mavros_msgs.srv.CommandTOL)
        self.SERVICE_LAND = TopicService("/mavros/cmd/land", mavros_msgs.srv.CommandTOL)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM = TopicService("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = TopicService("/mavros/param/get", mavros_msgs.srv.ParamGet)

        self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_BATTERY_STATE)
        self.topic_subscriber(self.TOPIC_GET_LOCAL_POSE)

    def arm(self, status):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result

    def takeoff(self, alt):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.altitude = alt
        data.latitude = 0
        data.longitude = 0
        data.min_pitch = 0
        data.yaw = 0
        self.SERVICE_TAKEOFF.set_data(data)
        result = self.service_caller(self.SERVICE_TAKEOFF, timeout=30)
        return result.success, result.result

    def change_mode(self, mode):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def move_global(self, x, y, z):
        if self.returning_to_home: return
        pose = geographic_msgs.msg.GeoPoseStamped()
        pose.header = std_msgs.msg.Header()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.latitude= x
        pose.pose.position.longitude= y
        pose.pose.position.altitude= z + 603.35
        self.TOPIC_SET_POSE_GLOBAL.set_data(pose)
        self.topic_publisher(topic=self.TOPIC_SET_POSE_GLOBAL)

    def move_local(self, x, y, z):
        if self.returning_to_home: return
        pose = geometry_msgs.msg.PoseStamped()
        pose.header = std_msgs.msg.Header()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x= x
        pose.pose.position.y= y
        pose.pose.position.z= z
        self.TOPIC_SET_POSE_LOCAL.set_data(pose)
        self.topic_publisher(topic=self.TOPIC_SET_POSE_LOCAL)

    def get_param(self, param):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.SERVICE_GET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param, value_integer, value_real):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        self.SERVICE_SET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_SET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real
    
    def return_to_home(self):
        print('urgent return home! battery: '+str(self.battery*100)+'%')
        self.move_local(self.home_x,self.home_y,self.z)
        self.returning_to_home = True
        time.sleep(20)
        self.change_mode(MODE_LAND)
    
    def update_parameters_from_topic(self):
        while True:
            if self.connected:
                #state
                data = self.TOPIC_STATE.get_data()
                self.armed = data.armed
                self.mode = data.mode

                #battery
                data = self.TOPIC_BATTERY_STATE.get_data()
                if data!=None:
                    self.battery = data.percentage
                
                #local position
                data = self.TOPIC_GET_LOCAL_POSE.get_data()
                if data!=None:
                    self.x = data.pose.position.x
                    self.y = data.pose.position.y
                    self.z = data.pose.position.z