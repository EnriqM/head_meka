#!/usr/bin/env python

'''
Created on Jun 14, 2018

This simply test how to send data to Arduino with CNC shield
In the future, integration with TF is intended as well as a model 
of the head.

Now, it works along headMeka_ArduinoROS.ino

@author: emortega
'''



# license removed for brevity
import rospy
#from std_msgs.msg import String
from head_meka_control.msg import HeadJoint 



def talkersteps():
    pub = rospy.Publisher('joint_steps', HeadJoint, queue_size=10)
    rospy.init_node('publishSteps')#, anonymous=True)
    rate = rospy.Rate(10) # Hz
    pasos = HeadJoint()
    posicion = 0
    pasito = 100
    while not rospy.is_shutdown():
        if posicion > 3600 or posicion < 0:
            pasito = -pasito
        
            
        pasos.position1 = posicion
        pasos.position2 = -posicion
        pub.publish(pasos)
        posicion = posicion + pasito
        #rospy.loginfo(pasos) #just to check if everything is OK
        rate.sleep()

if __name__ == '__main__':
    try:
        talkersteps()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
    
    '''def sumaposiciones(mensaje):
    
    if posicion > 3600:
        posicion = 0
    
    mensaje.position1 = 
    mensaje.position2
    
    return mensaje'''