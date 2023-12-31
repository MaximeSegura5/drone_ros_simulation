import rospy
from topicService import TopicService


class RosHandler(object):
    def __init__(self):
        self.rate = 1
        self.connected = False

    def connect(self, node, rate):
        rospy.init_node(node, anonymous=True)
        self.rate = rospy.Rate(rate)
        self.connected = True
        rospy.loginfo("Rospy is up ...")
        rospy.spin()
        return

    def disconnect(self):
        if self.connected:
            rospy.loginfo("shutting down rospy ...")
            rospy.signal_shutdown("disconnect")
            self.connected = False
        return

    @staticmethod
    def topic_publisher(topic):
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=1, latch=True)
        pub.publish(topic.get_data())

    @staticmethod
    def topic_subscriber(topic):
        rospy.Subscriber(topic.get_name(), topic.get_type(), topic.set_data)

    @staticmethod
    def service_caller(service, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            rospy.loginfo("waiting for ROS service:" + srv)
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo("ROS service is up:" + srv)
            call_srv = rospy.ServiceProxy(srv, typ)
            return call_srv(data)
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None

