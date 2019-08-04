#!/usr/bin/env python

import rospy
import numpy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32

class Laser2PC():

    def __init__(self):
        self.laserProj=LaserProjection()
        #self.pcPub=rospy.Publisher("/laserPointCloud",pc2,queue_size=1)
	self.laDistance=rospy.Publisher("/laser_distance",Float32,queue_size=1)
	self.laSector=rospy.Publisher("/laser_sector",Float32,queue_size=1)
	self.laserSub = rospy.Subscriber("/scan",LaserScan,self.laserCallback)

    def laserCallback(self,data):
	rangesCenter = numpy.zeros(20);
	rangesRight = numpy.zeros(20);
	rangesLeft = numpy.zeros(20);
        sector = 3.0;
	cavg = 3;
	ravg = 3;
	lavg = 3;
	accuracy = 2;
        #cloud_out = self.laserProj.projectLaser(data)
	#sector center
	for x in range(20):
	    rangesCenter[x] = round(data.ranges[70+x],2);
	for y in range(20):
	    rangesRight[y] = round(data.ranges[49 + y],2);
	    rangesLeft[y] = round(data.ranges[91 + y],2);
	indxc = numpy.where(rangesCenter < 1.0);
	indxr = numpy.where(rangesRight < 1.0);
	indxl = numpy.where(rangesLeft < 1.0);
	c = rangesCenter[indxc];
	r = rangesRight[indxr];
	l = rangesLeft[indxl];
	if c.size > accuracy: #right sector 1
	    cavg =  numpy.average(c);
	    sector = 0;    
	    print cavg;        
	    self.laDistance.publish(cavg);
    	    self.laSector.publish(sector);
	elif r.size > accuracy: #left sector 1
	    ravg =  numpy.average(r);
	    sector = 1;    
	    print ravg;        
	    self.laDistance.publish(ravg);
    	    self.laSector.publish(sector);
	elif l.size > accuracy: #left sector 1
	    lavg =  numpy.average(l);
	    sector = -1;    
	    print lavg;        
	    self.laDistance.publish(lavg);
    	    self.laSector.publish(sector);
	else:
 	    davg = 3;	       
	    dsector = 3.0;
	    print davg; 	
	    self.laDistance.publish(davg);
	    self.laSector.publish(dsector);

if __name__=='__main__':
    rospy.init_node("laser")
    l2pc=Laser2PC()
    rospy.spin()




