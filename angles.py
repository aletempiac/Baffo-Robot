#!/usr/bin/python

from random import randint

gamma=range(0,360,10)
beta=range(0,360,10)
all_angle_test=[]
all_angle_check=[]
for element in gamma:
	angle_test=[(angle-element+270) % 360 for angle in beta]
	angle_check=[270-a if a>180 else 90-a for a in angle_test]
	all_angle_test.append(angle_test)
	all_angle_check.append(angle_check)

for x, y in zip(all_angle_test, all_angle_check):
	#print x, y
	for k, v in zip(x,y):
    		if (x>=0) and (x<=90):
    			if (v<0):
    				print "Error %d %d" %(k, v)
    		elif (x>90) and (x<=180):
    			if (v>0):
    				print "Error %d %d" %(k, v)
    		elif (x>180) and (x<=270):
    			if (v<0):
    				print "Error %d %d" %(k, v)
    		elif (x>270) and (x<=360):
    			if (v>0):
    				print "Error %d %d" %(k, v)
