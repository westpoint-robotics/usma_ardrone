#!/usr/bin/env python

# Import required Python code.
import cv2
import roslib
import rospy
import yaml
import tf
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from robot_plugins.msg import pid_controllers

import csv
import itertools

class gazebo_logger():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):

		exp_date = rospy.get_param('~date')
		str_date = str(exp_date)
		print "gazebo_logger:: date:= " +str_date

		exp_run = rospy.get_param('~run')
		str_run = str(exp_run).zfill(3)
		print "gazebo_logger:: run:= " + str_run
		
		self.file_loggerfilename = '/home/benjamin/ros/data/csv/hast_' + str_date + '_' + str_run + '.csv'
		self.file_logger = open(self.file_loggerfilename, 'w')
		self.world_origin = rospy.get_param('~world_origin')
		xf_list = rospy.get_param('~xf_array')
		xf_names = rospy.get_param('~xf_names')
		self.array_of_xf = xf_list.split(",")
		self.array_of_xf_names = xf_names.split(",")

		metronome_type = rospy.get_param('~metronome_type')
		metronome_topic = rospy.get_param('~metronome_topic')
		self.Frame = 0
		if metronome_type == 'JointState':
			self.metronome_sub = rospy.Subscriber(metronome_topic,JointState,self.metronome_callback)
		elif metronome_type == 'Imu':
			self.metronome_sub = rospy.Subscriber(metronome_topic,Imu,self.metronome_callback)

		self.cfg_loggername = '/home/benjamin/ros/data/csv/hast_cfg_' + str_date + '_' + str_run + '.csv'
		self.cfg_logger = open(self.cfg_loggername, 'w')
		self.cfg_logger.write(str_date+'\n')
		self.cfg_logger.write(str_run+'\n')


		self.make_csv_headers()
		for xf_item in self.array_of_xf:
			print "gazebo_logger::logging xf : " + str(self.world_origin) + " to " + str(xf_item)
			self.cfg_logger.write("gazebo_logger::logging xf : " + str(self.world_origin) + " to " + str(xf_item)+'\n')

		self.xf_listener = tf.TransformListener()

		record_pid = rospy.get_param('~record_pid')
		if record_pid:
			self.pid_filename = '/home/benjamin/ros/data/' + str_date + '/' + str_run + '/pid_' + str_run + '.m'
			self.pid_logger = open(self.pid_filename, 'w')
			self.pid_topic = rospy.get_param('~pid_topic')
			print "gazebo_logger:: pid_logger:= " + self.pid_filename
			print "gazebo_logger:: pid_topic:= " + self.pid_topic
			self.pid_sub = rospy.Subscriber(self.pid_topic,pid_controllers,self.pid_callback)
			self.pid_filealloc = '/home/benjamin/ros/data/' + str_date + '/' + str_run + '/pid_prealloc_' + str_run + '.m'
		else:
			print "gazebo_logger:: not logging gazebo PID "

		while not rospy.is_shutdown():
			rospy.spin()

		self.RUN = str_run
		self.DATE = str_date
		print "self.processVicon()"
		self.processVicon()
		
		if record_pid:
			self.pid_callalloc()

	def processVicon(self):
		# PATH = sys.argv[1]
		# DATE = sys.argv[2]
		# RUN = sys.argv[3]
		OUTPATH = "/home/benjamin/ros/data/" + self.DATE + "/csv/"

		datafile = self.file_loggerfilename
		print datafile
		f = open(datafile, 'rt')
		datalist = []
		i = 0;
		try:
		    reader = csv.reader(f)
		    for row in reader:
		    	i += 1
		    	if i == 3:
			        objects = row
		        if i > 5:
			        datalist.append(row)
		finally:
		    f.close()

		stripped_objects = filter(None, objects)
		filenames=[]
		for x in stripped_objects:
			y = x.split(":")
			# print y[1]
			filenames.append(OUTPATH+y[1]+ '_' + self.RUN + ".csv")

		# print filenames
		for x in range(0, len(filenames)):
			f = open(filenames[x], 'wt')
			try:
			    writer = csv.writer(f)
			    writer.writerow( ('Frame','RX', 'RY', 'RZ', 'RW', 'TX', 'TY', 'TZ') )
			    for i in range(len(datalist)-1):
			        writer.writerow( [datalist[i][0]] + datalist[i][(2+7*x):(9+7*x)])
			finally:
			    f.close()

	def pid_callalloc(self):
		print " "
		print "gazebo_logger:: shutting down... "
		print " "
		self.file_logger.close()
		self.pid_logger.close()

		msg_id = self.pid_data.id
		self.pid_alloc = open(self.pid_filealloc, 'w')
		self.pid_alloc.write("%% pid preallo\n\n")
		self.pid_alloc.write("pid.time = zeros(" + str(msg_id) + ",1);\n")
		for ctrl in self.pid_data.controllers:
			self.pid_alloc.write("\npid." + ctrl.name + ".gains.pid = zeros(" + str(msg_id) + ",3);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".errors.pid = zeros(" + str(msg_id) + ",3);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".input = zeros(" + str(msg_id) + ",1);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".state = zeros(" + str(msg_id) + ",1);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".dinput = zeros(" + str(msg_id) + ",1);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".output = zeros(" + str(msg_id) + ",1);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".limit = zeros(" + str(msg_id) + ",1);\n")
			self.pid_alloc.write("pid." + ctrl.name + ".time_constant = zeros(" + str(msg_id) + ",1);\n")
		self.pid_alloc.close()
		
	def pid_callback(self,data):
		self.pid_data = data
		stamp = data.stamp
		msg_id = data.id
		self.pid_count = msg_id
		self.pid_logger.write("\npid.time(" + str(msg_id) + ",1) = [" + str(stamp) + "];\n")
		for ctrl in data.controllers:
			self.pid_logger.write("pid." + ctrl.name + ".gains.pid(" + str(msg_id) + ",:) = [" + str(ctrl.gains.p) + ", " + str(ctrl.gains.i) + ", " + str(ctrl.gains.d) + "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".errors.pid(" + str(msg_id) + ",:) = [" + str(ctrl.errors.p) + ", " + str(ctrl.errors.i) + ", " + str(ctrl.errors.d) + "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".input(" + str(msg_id) + ",1) = [" + str(ctrl.input)+ "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".state(" + str(msg_id) + ",1) = [" + str(ctrl.state)+ "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".dinput(" + str(msg_id) + ",1) = [" + str(ctrl.dinput)+ "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".output(" + str(msg_id) + ",1) = [" + str(ctrl.output)+ "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".limit(" + str(msg_id) + ",1) = [" + str(ctrl.limit)+ "];\n")
			self.pid_logger.write("pid." + ctrl.name + ".time_constant(" + str(msg_id) + ",1) = [" + str(ctrl.time_constant)+ "];\n")

	def make_csv_headers(self):
		header_line1 = 'Objects'
		header_line2 = '100'
		header_line3 = ',,'
		header_line4 = 'Frame,Sub Frame,'
		header_line5 = ',,'
		for xf in self.array_of_xf_names:
			header_line3 += 'Global Angle (Quaternion) ' + xf + ':' + xf + ',,,,,,,'
			header_line4 += 'RX,RY,RZ,RW,TX,TY,TZ,'
			header_line5 += ',,,,mm,mm,mm,'
		self.file_logger.write(header_line1 + '\n')
		self.file_logger.write(header_line2 + '\n')
		self.file_logger.write(header_line3 + '\n')
		self.file_logger.write(header_line4 + '\n')
		self.file_logger.write(header_line5 + '\n')

	def metronome_callback(self,data):
		# using the metronome topic, record all xfs
		stamp = data.header.stamp
		time = stamp.to_sec()
		time_f = '%.4f' % time
		self.Frame += 1
		data_line = str(self.Frame) + ',0,'
		for xf in self.array_of_xf:
			# compute transform
			# print "gazebo_logger::logging xf : " + str(self.world_origin) + " to " + str(xf)
			try:
				(xf_t,xf_x) = self.xf_listener.lookupTransform(self.world_origin, xf, rospy.Time(0))
			except :
				xf_t = [0,0,0]
				xf_x = [0,0,0,0]
				# print "gazebo_logger::except on " + xf
				# print "gazebo_logger::logging " + xf + " as: " + str(xf_x[0]) + ',' + str(xf_x[1]) + ',' + str(xf_x[2]) + ',' + str(xf_x[3]) + ',' + str(xf_t[0]*1000) + ',' + str(xf_t[1]*1000) + ',' + str(xf_t[2]*1000) + ','
			# write data
			data_line += str(xf_x[0]) + ','
			data_line += str(xf_x[1]) + ','
			data_line += str(xf_x[2]) + ','
			data_line += str(xf_x[3]) + ','
			# xf_t * 1000 converts m to mm
			data_line += str(xf_t[0]*1000) + ','
			data_line += str(xf_t[1]*1000) + ','
			data_line += str(xf_t[2]*1000) + ','
		self.file_logger.write(data_line + '\n')

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('gazebo_logger')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        log = gazebo_logger()
    except rospy.ROSInterruptException: pass
