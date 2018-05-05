#Intelligent Robotic Systems
A course by Dr. Armin Biess

## Homework 3 - ROS part I
The homework consists of 2 parts:
* __2 Nodes playing ping pong__ - `nodeA` and `nodeB` listen on topics `/ping` and `/pong` and send received messages to other topic
  * When both nodes launch withing 1 second - the nodes will exchange 2 messages. When second node launched with delay - it will miss the first node's initial message and hence only one message will be exchanged
  * Any `String` message that will be published to one of the topics will enter the exchange.
* `key` node that listens to the keyboard and sends command velocity to `/turtle1/cmd_vel` topic.  

