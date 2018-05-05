#!/usr/bin/python
import rospy
import curses
# Disclaimer: curses part was taken from https://gist.github.com/claymcleod/b670285f334acd56ad1c
from geometry_msgs.msg import Twist

# Variables:
node_name = "key"
publish_topic = "/turtle1/cmd_vel"


class Node:
    """
    This is a main node, which will publish velocity commands
    """
    def __init__(self):
        rospy.init_node(name=node_name)
        self.p = rospy.Publisher(name=publish_topic, data_class=Twist, queue_size=10)


def display(stdscr):
    """
    This is the main function which is executed repeatedly.

    """
    global n  # We need the node handler to publish messages

    k = 0 # initial keypress

    # Initializing curses interface
    stdscr.clear()
    stdscr.refresh()
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)

    while k != ord('q'):   # Q key stops the node
        # Cleaning the screen and getting size
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        # Creating a velocity command message
        cmd = Twist()

        # Updating command according to key pressed
        if k == 119:  # W key
            cmd.linear.x = 1
            key = 'W'
        elif k == 115: # s key
            cmd.linear.x = -1
            key = 'S'
        elif k == 97:  # A key
            cmd.angular.z = 1
            key = 'A'
        elif k == 100:  # D key
            cmd.angular.z = -1
            key = 'D'
        else:
            key = 'something else...'

        # Publishing the command
        n.p.publish(cmd)


        # Curses user interface text
        title = "Turtlebot Control Node"[:width-1]
        subtitle = "Home Assignment in Intelligent Robotic Systems Course"[:width-1]
        keystr = ("Last key pressed: "+key)[:width-1]
        # keystr = "Last key pressed: {}".format(k)[:width-1]
        statusbarstr = "Press 'q' to exit"
        separator = '-' * 20
        info_msg = "WASD keys send velocity commands to the turtle."


        # Centering calculations
        start_x_title = int((width // 2) - (len(title) // 2) - len(title) % 2)
        start_x_subtitle = int((width // 2) - (len(subtitle) // 2) - len(subtitle) % 2)
        start_x_keystr = int((width // 2) - (len(keystr) // 2) - len(keystr) % 2)
        start_y = int((height // 3) - 2)

        # Render status bar
        stdscr.attron(curses.color_pair(3))
        stdscr.addstr(height-1, 0, statusbarstr)
        stdscr.addstr(height-1, len(statusbarstr), " " * (width - len(statusbarstr) - 1))
        stdscr.attroff(curses.color_pair(3))

        # Turning on attributes for title
        stdscr.attron(curses.color_pair(2))
        stdscr.attron(curses.A_BOLD)

        # Rendering title
        stdscr.addstr(start_y, start_x_title, title)

        # Turning off attributes for title
        stdscr.attroff(curses.color_pair(2))
        stdscr.attroff(curses.A_BOLD)

        # Print rest of text
        stdscr.addstr(start_y + 1, start_x_subtitle, subtitle)
        stdscr.addstr(start_y + 3, (width // 2) - len(separator)//2, separator)
        stdscr.addstr(start_y + 5, (width // 2) - len(info_msg) // 2, info_msg)

        stdscr.addstr(start_y + 10, start_x_keystr, keystr)

        # Refresh the screen
        stdscr.refresh()

        # Wait for next input
        k = stdscr.getch()


if __name__ == '__main__':
    # This executes on node run
    try:
        # Creating node
        global n
        n = Node()

        # Starting curses
        curses.wrapper(display)
    except rospy.ROSInterruptException:
        pass
