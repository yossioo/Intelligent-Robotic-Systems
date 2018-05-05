#!/usr/bin/python
# Necessary imports:
import rospy
from std_msgs.msg import String

# Parameters
node_name = "nodeB"
listen_topic = "ping"
publish_topic = "pong"
delay = 1


class DummyNode:
    """
    A node which relays messages from [listen_topic] to [] after [delay] seconds
    """
    def __init__(self):
        # Initializing node
        rospy.init_node(name=node_name)
        # Creating publisher
        self.p = rospy.Publisher(name=publish_topic, data_class=String, queue_size=10)
        # Creating subscriber
        rospy.Subscriber(name=listen_topic, data_class=String, callback=self.woof)

    def woof(self, data):
        """
        This is a callback function.
        1. Get the message
        2. Print the message
        3. Wait [delay]
        4. Publish the message

        :paramdata:
        :type data: String
        :return:
        :rtype:
        """
        print(data.data)
        rospy.sleep(delay)
        self.p.publish(data)


def main():
    # Create tho node
    n = DummyNode()

    # Wait a second and send a message
    rospy.sleep(1)
    n.p.publish(String(data="--pong--"))

    # Ah, ha, ha, ha, stayin' alive
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
