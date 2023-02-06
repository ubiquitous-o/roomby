#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('modified_teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from ca_msgs.msg import DefineSong
from ca_msgs.msg import PlaySong

import time

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j         l
   m    k    .

anything else : stop

q/z : high speed mode/low speed mode
c: circle move mode

Playing song(select roomba talking word)
---------------------------
   1,2,3,4

1 : "OK"
2 : "!!"
3 : "??"
4 : "yeah!"

CTRL-C to quit
"""

# KeyBindings
moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        'k':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        # 'O':(1,-1,0,0),
        # 'I':(1,0,0,0),
        # 'J':(0,1,0,0),
        # 'L':(0,-1,0,0),
        # 'U':(1,1,0,0),
        # '<':(-1,0,0,0),
        # '>':(-1,-1,0,0),
        # 'M':(-1,1,0,0),
        # 't':(0,0,1,0),
        # 'b':(0,0,-1,0),
    }

speedBindings={
        'c':(0.5,3),
        'q':(0.5, 1.2),
        'z':(0.15,0.8),
        # 'q':(1.1,1.1),
        # 'z':(.9,.9),
        # 'w':(1.1,1),
        # 'x':(.9,1),
        # 'e':(1,1.1),
        # 'c':(1,.9),
    }
songBindings={
        '1':0,
        '2':1,
        '3':2,
        '4':3,
    }


# Song Definition
## LOW tone of voice 
a_voice={
    # OK
    0:{
        'song': 0,
        'length': 3,
        'notes':  [68,30,64],
        'durations': [0.2,0.1,0.5]
    },
    # !!
    1:{
        'song': 1,
        'length': 4,
        'notes':  [62,69,62,69],
        'durations': [0.1,0.1,0.1,0.1]
    },
    # ??
    2:{
        'song': 2,
        'length': 4,
        'notes':  [57,52,51,56],
        'durations': [0.1,0.1,0.2,0.2]
    },
    # yeah!
    3:{
        'song': 3,
        'length': 5,
        'notes':  [72,72,30,72,77],
        'durations': [0.1,0.1,0.05,0.1,0.5]
    }
}

## HIGH tone of voice
b_voice={
    # OK
    0:{
        'song': 0,
        'length': 3,
        'notes':  [80,30,76],
        'durations': [0.2,0.1,0.5]
    },
    # !!
    1:{
        'song': 1,
        'length': 4,
        'notes':  [86,93,86,93],
        'durations': [0.1,0.1,0.1,0.1]
    },
    # ??
    2:{
        'song': 2,
        'length': 4,
        'notes':  [81,76,75,80],
        'durations': [0.1,0.1,0.2,0.2]
    },
    # yeah!
    3:{
        'song': 3,
        'length': 5,
        'notes':  [84,84,30,84,89],
        'durations': [0.1,0.1,0.05,0.1,0.5]
    }
}


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

class SongPublishThread(threading.Thread):
    def __init__(self, rate):
        super(SongPublishThread, self).__init__()
        self.define_publisher = rospy.Publisher('define_song', DefineSong, queue_size = 1)
        self.play_publisher = rospy.Publisher('play_song', PlaySong, queue_size = 1)
        self.condition = threading.Condition()
        self.done = False
        self.voice_tone = "a"

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()


    def define_allsong(self):
        song0 = DefineSong()
        song1 = DefineSong()
        song2 = DefineSong()
        song3 = DefineSong()
        songs = [song0, song1, song2, song3]
        
        if self.voice_tone == "a":

            for i in range(len(songs)):
                songs[i].song= a_voice[i]["song"]
                songs[i].length = a_voice[i]["length"]
                songs[i].notes = a_voice[i]['notes']
                songs[i].durations = a_voice[i]['durations']
                self.define_publisher.publish(songs[i])
                print('Now defining song no: %d'% int(i+1))
                time.sleep(1.0)


        elif self.voice_tone == "b":

            for i in range(len(songs)):
                songs[i].song= b_voice[i]["song"]
                songs[i].length = b_voice[i]["length"]
                songs[i].notes = b_voice[i]['notes']
                songs[i].durations = b_voice[i]['durations']
                self.define_publisher.publish(songs[i])
                print('Now defining song no: %d'% int(i+1))
                time.sleep(1.0)

    def play(self, key):
        self.play_publisher.publish(key)
        print('NOW PLAYING SONG ID : %d'% int(key+1))
        

    def stop(self):
        self.done = True
        self.join()
    
    def set_voice_tone(self, tone):
        if tone == 'a':
            self.voice_tone = "a"
            print("\n Your robot is A mode \n")
        else:
            self.voice_tone = "b"
            print("\n Your robot is B mode \n")
        self.define_allsong()

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
    

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('modified_teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.15)
    turn = rospy.get_param("~turn", 0.8)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)
    song_thread = SongPublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    
    print("Which tone of voice ?")
    print("Press \'a\' , LOWER voice.")
    print("Press \'b\' , HIGHER voice.")

    while(1):
        key = getKey(settings, key_timeout)
        if key == 'a':
            song_thread.set_voice_tone('a')
            break
        elif key == 'b':
            song_thread.set_voice_tone('b')
            break
    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speedBindings[key][0]
                turn = speedBindings[key][1]
                # speed = min(speed_limit, speed * speedBindings[key][0])
                # turn = min(turn_limit, turn * speedBindings[key][1])
                # if speed == speed_limit:
                #     print("Linear speed limit reached!")
                # if turn == turn_limit:
                #     print("Angular speed limit reached!")
                print(vels(speed,turn))
                # if (status == 14):
                #     print(msg)
                # status = (status + 1) % 15
            elif key in songBindings.keys():
                song_thread.play(songBindings[key])

            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        song_thread.stop()

        restoreTerminalSettings(settings)
