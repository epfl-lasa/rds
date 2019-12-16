#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rds_network_ros.msg import ToGui

import numpy as np

import math

import matplotlib.pyplot as plt

rds_runtime_sequence = []
rds_n_constraints_sequence = []

def mysplit(s):
	words = []
	inword = 0
	for c in s:
		if c in " \r\n\t": # whitespace
			inword = 0
		elif not inword:
			words = words + [c]
			inword = 1
		else:
			words[-1] = words[-1] + c
	return words

def qolo_callback(msg):
	global rds_runtime_sequence
	words = mysplit(msg.data)
	rds_runtime_sequence.append(float(words[2]))

def to_gui_callback(msg):
	global rds_n_constraints_sequence 
	n_constraints = np.size(msg.solver_constraints)
	if n_constraints > 0:
		n_constraints = n_constraints/np.size(msg.solver_constraints[0])
	rds_n_constraints_sequence.append(n_constraints)

def average(sequence):
	a = 0.0
	for x in sequence:
		a = a + x
	return a/len(sequence)

def standard_deviation(sequence):
	a = average(sequence)
	sd = 0.0
	for x in sequence:
		sd = sd + (x - a)*(x - a)
	sd = math.sqrt(sd/(len(sequence) - 1))
	return sd

def main():
	rospy.init_node('rds_statistics')
	rospy.Subscriber("qolo", String, qolo_callback)
	rospy.Subscriber("rds_to_gui", ToGui, to_gui_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
	print "average rds runtime:"
	print average(rds_runtime_sequence)
	print "standard deviation rds runtime:"
	print standard_deviation(rds_runtime_sequence)
	plt.plot(rds_runtime_sequence)
	plt.title("RDS Runtime")
	plt.show()
	print "average number of constraints:"
	print average(rds_n_constraints_sequence)
	print "standard deviation number of constraints:"
	print standard_deviation(rds_n_constraints_sequence)
	plt.plot(rds_n_constraints_sequence)
	plt.title("RDS number of constraints")
	plt.show()