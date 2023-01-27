#! /usr/bin/env python3
# Nodo para la prueba: Navegacion SIN obstaculos
# Equipo: dotMEX-CAR 2022
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64

bridge = CvBridge()
FT = 0
l = 80 #50 
x_ref = 120
x1 = 120
x2 = 120
x1_h = 120
u = 0.0
v = 55.0 #55.0
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def tip(imagenN):
	H=np.array([[-5.88392618e-02,-4.02514041e-01,1.19565927e+02],[1.24049432e-18,-1.34260497,3.67342070e+02],[4.47714719e-21,-4.01176785e-03,1.0]])  #Banqueta
	imagenH = cv2.warpPerspective(imagenN, H, (200,300),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def roi_zone(x):
	assert (x>=0) and (x<=199), 'x out of limits'
	if (x>120) and (x<=199):
		y = int(round(-1.7722*x+511.6582))
	if (x>=80) and (x<=120):
		y = 299
	if (x>=0) and (x<80):
		y = int(round(1.6875*x+164.0))
	return y
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def vec_create(x,stride,side):
	"""
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	"""
	if(side==1):
		xi = x+stride
		xd = x-stride
	else:
		xi = x-stride
		xd = x+stride
	if(xi<0): xi = 0
	if(xi>199): xi = 299
	if(xd<0): xi = 0
	if(xd>199): xi = 299
	xv = np.arange(xi,xd,(-1)*side)
	return xv
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def line_detector(imagen0,x1,l,side):
	K = True
	stridex = 3
	stridey = 5
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stridex,side)
	while (K==True):
		m = y1+stridey
		if (m>=299): m = 299
		for j in range(m,y1-stridey,-1):
			for i in x1v:
				if imagen0[j][i]==255:
					x1 = i
					y1 = j
					K = False
					break
			x1v = vec_create(x1,stridex,side)
			if (K==False): break
		if (K==True): 
			x1 = x1-1*side
			y1 = roi_zone(x1)

	x2 = x1
	x2v = vec_create(x2,stridex,side)
	for j in range(y1-1,y1-l,-1):
		for i in x2v:
			if imagen0[j][i]==255:
				x2 = i
				y2 = j
				K = False
				break
		x2v = vec_create(x2,stridex,side)			
	return x1,y1,x2,y2
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_V(data0):
	global u, v
	global FT 
	global x1, x2, x1_h

	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 	
	imagenF = tip(imagen0)	
	lower1 = np.array([10,0,100]) 
	upper1 = np.array([30,100,150]) 
	imagenF1 = cv2.inRange(cv2.cvtColor(imagenF,cv2.COLOR_BGR2HSV),lower1,upper1)
	lower2 = np.array([0,0,100]) 
	upper2 = np.array([179,50,255]) 
	imagenF2 = cv2.inRange(cv2.cvtColor(imagenF,cv2.COLOR_BGR2HSV),lower2,upper2)
	imagenF = cv2.GaussianBlur(imagenF1+imagenF2,(9,9),0)				
	_,imagenF = cv2.threshold(imagenF,25,255,cv2.THRESH_BINARY)
	imagenF = cv2.Sobel(imagenF, cv2.CV_8U, 1, 0, ksize=7, borderType=cv2.BORDER_DEFAULT)

	y1 = 0
	y2 = 0
	if (FT<=30):
		x1 = 180
		FT = FT+1
	else: 
		x1 = x1_h
	x1,y1,x2,y2 = line_detector(imagenF,x1,l,1)
	x1_h = x1

	# CONTROL
	ky = 0.02143299
	kth = 0.25015021

	e_y = x1-x_ref
	e_th = np.arctan2(x2-x1,l)
	u = np.arctan(ky*e_y+kth*e_th)
	if (u>0.83): u = 0.83
	if (u<-0.83): u = -0.83
	Vpub.publish(15) 
	Spub.publish(0)

	#Visualizacion 2021b
	print('e_y: = ', e_y) #-2 a (-1)
	print('e_th: = ', e_th) #0.0
	print('u: = ', u) #-0.012 a -0.02
	#print('x1: = ', x1) #119
	#print('x2: = ', x2) #119
	#print('y1: = ', y1) #298
	#print('y2: = ', y2) #219
	
	imagenF = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
	imagenF = cv2.circle(imagenF,(x1,y1),3,(0, 0, 255),-1)
	imagenF = cv2.circle(imagenF,(x2,y2),3,(0, 0, 255),-1)
	imagenF = cv2.line(imagenF, (x1,y1), (x2,y2), (0, 0, 255), 3)
	
	cv2.imshow('homografia',imagenF)	
	#cv2.moveWindow("homografia", 400,20)
	cv2.waitKey(1)
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_01.py")
	rospy.init_node('TMR_01',anonymous=True)												
	Vpub = rospy.Publisher('speed',Float64,queue_size=15)				 
	Spub = rospy.Publisher('steering',Float64,queue_size=15)
	rospy.Subscriber("/camera/rgb/raw",Image,callback_V)	 				
	rospy.spin()
