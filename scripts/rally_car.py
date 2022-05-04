#!/usr/bin/env python
'''
Author:Harikrishnan Manoj Krishnan
DOC:08/12/2020
UPDATE:04/22/2022 
Mobile Robots and Knoy500 Race: This script uses AMCl for localization of the rally car. The goal waypoints are generated using get_waypoints.py script and stored in a text file.
              Load waypoints(), setup 2d Pose estimation from rviz to provide current position based on the map and run this script. The rally car will follow the goal way_points

Final Version
'''



import rospy
import numpy as np
import serial
import time
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib import pyplot as plt # For graph
class rally_car():
    def __init__(self):
        #PID Parameters lin --> Linear motion; ang --> angular motion; err --> error; perr --> previous error;
        self.linkp= 180
        self.linkd= 200
        self.angkp= 0.5
        self.angkd= 5
        self.rate= 100
        self.linerr= 0
        self.plinerr= 0
        self.angerr= 0
        self.pangerr=0
        self.thres= 2 # Threshold value for updating the points
        self.dat= 0
        self.time_now = 0
        self.time_now1 = 0
        self.prev_time = 0 
        self.dt = 0
        self.count = 0  #Cycle count
        self.speed = 70 # To set the speed
        self.spd_limit = 0 # Speed Theshold

        

        # Velocity data
        self.linvel = 0 #linear
        self.angvel = 0 # Steer

        # AMCL data
        self.mov_x = 0 #Current X
        self.mov_y = 0 #Current Y
        self.mov_yaw = 0 #Current Z 
    
        # Initialize way - points
        #self.list = np.zeros((2,2))
        self.i = 0
        self.nxt_x = 0
        self.nxt_y = 0
        self.nxt_yaw = 0
        self.pt_upd = 0 #Timer count for updating the way-points
        
        #Serial data
        self.angdat = 0
        self.lindat = 0
        self.console = serial.Serial('/dev/ttyACM0', baudrate = 115200)
        if self.console.isOpen() == False:
            self.console.isOpen()

        # AMCL data for localizing
    def amcl_data(self, data):
        self.mov_x = data.pose.pose.position.x
        self.mov_y = data.pose.pose.position.y
        yaw = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.mov_yaw = yaw[2]
        
    #To convert data for arduino(Serial data for linear speed)
    def serial_conv_lin(self,inp):
        output = ((inp-0)*(2048-0)/(2048-0)+2048)
        if(inp<0):
            output = 0
            return output
        elif(inp>2048):
            output = 2048
            return output
        else:
            return output
    #To convert data for arduino(Serial data for steer speed)    
    def serial_conv_ang(self,inp):
        output = ((inp-(-2048)*(2048-(-2048)/(2048-(-2048))+2048)
        if(inp<-2048):
            output = -2048
            return output
        elif(inp>2048):
            output = 2048
            return output
        else:
            return output
        
    def main(self):
        #Ros intialization
        rospy.init_node('rallyrun', anonymous = True)
        rospy.loginfo("Racing time!!")
        rate = rospy.Rate(self.rate)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_data)
        list1 = np.loadtxt("/home/crl5/rally_ws/src/wall_follower/resources/waypoints.txt")
        while not rospy.is_shutdown():
            self.linerr = np.sqrt(((self.mov_x - self.nxt_x)**2) + ((self.mov_y- self.nxt_y))**2)
            angle = math.atan2((self.nxt_y - self.mov_y),(self.nxt_x-self.mov_x))
            self.angerr = angle - self.mov_yaw
     
            if (self.linerr<self.thres) and ((rospy.get_time() - self.pt_upd)>0.1):
                self.nxt_x = list1[self.i,0]
                self.nxt_y = list1[self.i,1]
                self.pt_upd = rospy.get_time()
                if(self.i<((len(list1))-1)):
                    self.i = self.i+1 

            self.time_now = time.time()
            #self.dt = self.time_now - self.prev_time
                    
            #PD contorller
            self.lindat = self.linkp*self.linerr + self.linkd*((self.linerr - self.plinerr))
            
            if (self.angerr > math.pi):
                self.angerr = self.angerr + (-math.pi*2)
            elif (self.angerr< -math.pi):
                self.angerr = self.angerr + (math.pi*2)
        
            self.angdat = self.angkp*self.angerr + self.angkd*((self.angerr - self.pangerr))
            self.plinerr = self.linerr
            self.pangerr = self.angerr
            
            #Plotting graph Linear_error(x) vs time(y)
            l_y = self.linerr
            l_x = self.time_now
            plt.subplots(1,2,1)
            plt.plot(l_x,l_y,marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
            plt.x_label("Linear_error")
            plt.y_label("time(ms)")
            #Plotting graph steering_error(x) vs time(y)
            a_y = self.linerr
            a_x = self.time_now
            plt.subplots(1,2,2)
            plt.plot(a_x,a_y,marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
            plt.x_label("Linear_error")
            plt.y_label("time(ms)")
            plt.show()            

            
            linear = self.serial_conv_lin(self.lindat)
            angular = self.serial_conv_ang(self.angdat)                 
            #print(linear)
            
            # Providing data to arduino
            self.spd_limit = 1.0*self.speed/100
            self.linvel = int(linear)*spd_limit
            self.angvel = int(angular)
            #print(self.linvel)
            if(abs(self.angvel)>1800):
                self.count +=1   
                if(self.count == 10):#For every 10 cycles
                    self.count=0
                    self.linvel = 0
                else:
                    self.linvel = 500*self.spd_limit
            
            if(self.linvel>440):
                self.linvel = 440*self.spd_limit

            self.dat = ('A'+'%+05d' %self.angvel+'%+05d' %self.linvel)
            self.console.write('A'+'%+05d' %self.angvel+'%+05d' %self.linvel)
            print(self.dat)
            rate.sleep()
            
                
        if rospy.is_shutdown():
            self.console.write('A+0000+0000')
     
            

if __name__=="__main__":
    try:
        obj = rally_car()
        obj.main()
    except rospy.ROSInterruptException:
        pass

            
           
