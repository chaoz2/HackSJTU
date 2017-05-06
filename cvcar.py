#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os
from Tkinter import *
import cv2
import time
from PIL import Image
from PIL import ImageTk
import roslib;
roslib.load_manifest('cvcar')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import multiprocessing
import subprocess 
from subprocess import Popen,PIPE
import threading


class Teleop:

	pub = rospy.Publisher('arduino', Twist, queue_size=1)

	def __init__(self):		
		# rospy.init_node('turtlesim_node',anonymous=True)
		rospy.init_node('cvcar', anonymous=True)
#		rate = rospy.Rate(rospy.get_param('~hz', 1)) 
#		topic = rospy.get_param('~topic', '/turtle1/cmd_vel')
#		print topic

    def __del__(self):
        # os.system('sudo pkill roscore')
        # os.system('sudo pkill rosmaster')
        # os.system('sudo pkill avrdude')
        os.system('sudo pkill rosout')
                  
	def up(self):
		cmd = Twist()
		cmd.linear.x = 2.0
		cmd.linear.y=0.0
		cmd.linear.z=0.0
		cmd.angular.x=0.0
		cmd.angular.y=0.0
		cmd.angular.z=0.0
		self.pub.publish(cmd)

	def left(self):
		cmd = Twist()
		cmd.linear.x = 6.0
		cmd.linear.y=0.0
		cmd.linear.z=0.0
		cmd.angular.x=0.0
		cmd.angular.y=0.0
		cmd.angular.z=0.0
		self.pub.publish(cmd)

	def right(self):
		cmd = Twist()
		cmd.linear.x = -6.0
		cmd.linear.y=0.0
		cmd.linear.z=0.0
		cmd.angular.x=0.0
		cmd.angular.y=0.0
		cmd.angular.z=0.0
		self.pub.publish(cmd)

	def back(self):
		cmd = Twist()
		cmd.linear.x = -2.0
		cmd.linear.y=0.0
		cmd.linear.z=0.0
		cmd.angular.x=0.0
		cmd.angular.y=0.0
		cmd.angular.z=0.0
		self.pub.publish(cmd)

    '''
    def shift(self):
            cmd = Twist()
            cmd.linear.x = 1.0
            cmd.linear.y=0.0
	cmd.linear.z=0.0
	cmd.angular.x=0.0
	cmd.angular.y=0.0
	cmd.angular.z=0.0
	self.pub.publish(cmd)
    '''

    def space(self): 
    	cmd = Twist()
    	cmd.linear.x = 1.0
   	    cmd.linear.y = 0.0
		cmd.linear.z = 0.0
		cmd.angular.x = 0.0
		cmd.angular.y = 0.0
		cmd.angular.z = 0.0
		self.pub.publish(cmd)


	def listener(self):

		# don't init, just subscribe a new topic
		rospy.Subscriber("chatter", String, callback)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()


global is_run
is_run=False

def callback(data):
	#rospy.loginfo("I heard %s",data.data)
	global is_run
	if is_run is True:
		return

	if data.data == 'ship':
		is_run=True
		rospy.loginfo("I heard %s", data.data)
		upClick()
		time.sleep(2)
		spaceClick()
		is_run=False

	elif data.data == 'cat':
		is_run=True
		rospy.loginfo("I heard %s", data.data)
		backClick()
		time.sleep(2)
		spaceClick()
		is_run=False

	elif data.data == 'chair':
		is_run=True
		rospy.loginfo("I heard %s", data.data)
		leftClick()
		time.sleep(3)
		spaceClick()
		is_run=False

	elif data.data == 'bicycle':
		'''
		is_run=True
		rospy.loginfo("I heard %s",data.data)
		rightClick()
		time.sleep(2)
		spaceClick()
		spaceClick()
		rospy.loginfo("sleep 2s")
		leftClick()
		time.sleep(2)
		rospy.loginfo("sleep 4s")
		spaceClick()
		spaceClick()
		is_run=False
		'''
		is_run=True
		rospy.loginfo("I heard %s",data.data)
		rightClick()
		time.sleep(3)
		spaceClick()
		is_run=False

	elif data.data == 'car':
		is_run=True
		rospy.loginfo("I heard %s",data.data)
		leftClick()
		time.sleep(0.8)
		upClick()
		time.sleep(2)
		spaceClick()
		is_run=False

	elif data.data == 'train':
		is_run=True
		rospy.loginfo("I heard %s",data.data)
		rightClick()
		time.sleep(0.8)
		upClick()
		time.sleep(2)
		spaceClick()
		is_run=False

	else :
		spaceClick()

t=Teleop()
def upClick():	
	t.up()
def leftClick():	
	t.left()
def rightClick():	
	t.right()
def backClick():	
	t.back()
'''
def shiftClick():
    t.shift()
'''
def spaceClick(): 
    t.space()
def upKey(event):
	upClick()
def leftKey(event):
	leftClick()
def rightKey(event):
	rightClick()
def backKey(event):
	backClick()
'''
def shiftKey(event):
    shiftClick()
'''
def spaceKey(event):
    spaceClick()

def openVideo():
	os.system('rosrun sjtuhack camera.py')

def videoClick():
	t = threading.Thread(target=openVideo)
	t.setDaemon(True)
	t.start()
	#subprocess.Popen("rosrun ros_caffe ros_caffe_test  &",shell = True,stdlin = PIPE,stdout =PIPE,stderr = PIPE)
	
if __name__ == '__main__':
	'''
	os.system('roscore &')
	os.system('sleep 3s')
	
	os.system('rosrun rosserial_python  serial_node.py  /dev/ttyACM0 &')
	os.system('sleep 3s')
	'''
	listenThread=threading.Thread(target=t.listener)
	#t.listener()
	listenThread.setDaemon(True)
	listenThread.start()

	root=Tk()
	f=Frame(root, height=256, width=256)

	f.bind('<Up>',upKey)
	f.bind('<Left>',leftKey)
	f.bind('<Right>',rightKey)
	f.bind('<Down>',backKey)
	# f.bind('<Shift_L>',shiftKey)
	f.bind('<space>',spaceKey)
	f.focus_set()
	f.pack()
	# 界面生成
	top=Button(f,text='up', width=8, height=8, command=upClick)
	left=Button(f,text='left',width=8,height=8,command=leftClick)
	right=Button(f,text='right',width=8,height=8,command=rightClick)
	back=Button(f,text='down',width=8,height=8,command=backClick)
    # shift=Button(f,text='shift',width=8,height=8,command=shiftClick)
	space=Button(f,text='space',width=8,height=8,command=spaceClick)
	videoButton = Button(f,text='video',width=8,height=8,command=videoClick)
    # shift.grid(row=2,column=0)
	space.grid(row=2,column=0)
	top.grid(row=1,column=2)
	left.grid(row=2,column=1)
	right.grid(row=2,column=3)
	back.grid(row=2,column=2)
	videoButton.grid(row=0,column=0)
	root.mainloop()
