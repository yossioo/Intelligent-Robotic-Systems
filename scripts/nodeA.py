#!/usr/bin/python
import rospy
from std_msgs.msg import String

node_name = "nodeA"
listen_topic = "pong"
publish_topic = "ping"


class DummyNode:
    def __init__(self):
        self.n = rospy.init_node(name=node_name)
        self.p = rospy.Publisher(name=publish_topic, data_class=String, queue_size=10)
        self.ns = rospy.get_namespace()
        rospy.Subscriber(name=listen_topic,data_class=String, callback=self.woof)
        dummy_msg = String()
        dummy_msg.data = "A message"
        self.p.publish(dummy_msg)

    def woof(self, data):
        """

        :paramdata:
        :type data: String
        :return:
        :rtype:
        """
        # print("Received on [/{}]:".format(listen_topic))
        print(data.data)
        rospy.sleep(1)
        # print("Sending to [/{}]".format(publish_topic))
        self.p.publish(data)


def main():
    n = DummyNode()
    rospy.sleep(1)
    n.p.publish(String(data="--ping--"))
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except:
        pass
