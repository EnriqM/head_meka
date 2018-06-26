#!/usr/bin/env python

'''
Created on Jun 21, 2018

This simply test how to send data to Arduino with CNC shield
In the future, integration with TF is intended as well as a model 
of the head.

Now, it works along headMeka_ArduinoROS.ino

@author: emortega
'''



# license removed for brevity
import rospy
#from std_msgs.msg import String
from math import pi
from head_meka_control.msg import HeadJoint 
from sensor_msgs.msg import JointState

class Control(object):
    def __init__(self):
        rospy.init_node('publishSteps')
        self.pub = rospy.Publisher('joint_steps',  HeadJoint, queue_size=10)
        rospy.Subscriber('/joint_states_head', JointState , self.update_value)
    
        #Variables:
        self.pasos = HeadJoint()
        micro_rev = 3600.0
        micro_rev1 = 800.0
        self.stepsPerRev = [3.0*micro_rev, 1.0*micro_rev1]
        print "los pasos por revolucion son " + str(self.stepsPerRev)
        self.angles = [0.0, 0.0]
        self.prev_angle = self.angles
        #self.pasos.position1 = posicion
        #self.pasos.position2 = -posicion
        self.flag = False
        
    def update_value(self, msg):
        #self.value = msg.data + 1
        #print self.value
        if msg.position != self.prev_angle:
            self.angles = msg.position
            self.prev_angle = self.angles
            self.flag = True
            rospy.loginfo("New angle listened")
        #print "Posicion 0 es: " + str(msg.position[0])
        #print "Posicion 1 es: " + str(msg.position[1])
    
    def convert(self, angle):
        steps = [0.0, 0.0]
        steps[0] = (angle[0] * self.stepsPerRev[0])/(2*pi)
        steps[1] = (angle[1] * self.stepsPerRev[1])/(2*pi)
        return steps
        
    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            try:
                if self.flag:
                    self.flag = False
                    vec_steps = self.convert(self.angles)
                    self.pasos.position1 = int(vec_steps[0])
                    self.pasos.position2 = int(vec_steps[1])
                    self.pub.publish(self.pasos)
                    rospy.loginfo("Vector publicado es: " + str(self.pasos))
                r.sleep()
            except (rospy.ROSInterruptException):
                pass
    
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
        foo = Control()
        foo.run()
    except rospy.ROSInterruptException:
        pass


'''
import rospy
from std_msgs.msg import Int32

class Echo(object):
    def __init__(self):
        self.value = 0

        rospy.init_node('echoer')
        self.pub = rospy.Publisher('/value', Int32, latch=True)
        rospy.Subscriber('/value', Int32, self.update_value)

    def update_value(self, msg):
        self.value = msg.data + 1
        print self.value

    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown():
        while not self.value == 40:
            self.pub.publish(self.value)
            r.sleep()

if __name__ == '__main__':
    prueba = Echo()
    prueba.run()
    
    
'''    


    
