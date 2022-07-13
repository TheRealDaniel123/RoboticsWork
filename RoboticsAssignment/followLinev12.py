
#Imports the required libraries
import numpy

import time

import cv2
import cv_bridge
import rospy
import math

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



class Follower:
    def __init__(self):


        #Initally assigns stop to false
        self.stop = False

        #OpenCv bridge for colour detection
        self.bridge = cv_bridge.CvBridge()

        #Subscibes to the laser scan topic
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
    
        #Subscibes to the image topic
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)

        #Subscribes to the odometry topic
        self.odometry = rospy.Subscriber('/odom',Odometry,self.callback_odometry)

        #Publishes to the velocity topic
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                                                    queue_size=1)

        
        #X position used for odometry
        self.firstXPosition = 0 

        #Max counter variable to help get the robot out of being stuck
        self.maxCounter = 700

        #Counter variable to count up to max counter
        self.counter = 0

        #Twist variable used to adjust the direction and orientation of the robot               
        self.twist = Twist()

        #Time variables for time delays
        self.currentTime = 0
        self.laserOffTime = 0
        
        #Sets whether the laser is on or off
        self.laserOn = True

        self.stoppingTime = 0

        self.frontDistance = 0
        self.leftDistance = 0
        self.rightDistance = 0

        self.rightDistanceRangeToLeft = 0

    

    #Odometry function used to get the robot out of being stuck
    def callback_odometry(self,odementry_msg):

 
      
        #If the counter has incremented enough rotate the robot 
        if self.counter == self.maxCounter:
            
            self.laserOn = False
            self.turnLeftOrRight(180)
            self.stopMethod
            

        else:


            tempXPos = self.firstXPosition
            if self.stop == True:
                self.firstXPosition = odementry_msg.pose.pose.position.x #Get the robots current x position
               
                if self.firstXPosition == tempXPos: #If the robots x position hasnt changed

                    self.counter += 1 #Increment the counter


    #Robots laser function     
    def callback_laser(self,laser_msg):

        #Left, Right and Front laser variables
        self.leftDistance = laser_msg.ranges[630]
        self.rightDistance = laser_msg.ranges[10]
        self.frontDistance = laser_msg.ranges[320]

        self.rightDistanceRangeToLeft = min(laser_msg.ranges[10:630])
       
        


        #Current time variable
        self.currentTime = time.time()

        #Set the laser back on after looking at red
        if self.currentTime - self.laserOffTime > 1:
            self.laserOn = True
     
        
        #If laser is switched on
        if self.laserOn: 

            #If there isn't a wall infront of the front laser
            if self.frontDistance > 1 and self.rightDistanceRangeToLeft > 0.5:

                #Time that the robots been moving, used to add a time delay when detecting blue
                self.moveingTime = time.time()


                #Adds a time delay to prevent the robot from getting stuck looking at blue
                if self.moveingTime - self.stoppingTime > 1:

                    self.stop = False

                #Calculate an error value to move the robot smoothly
                error = self.rightDistance - self.leftDistance
                self.moveForward(error,10) #Move forwards using the error
                self.counter = 0 #Set counter to 0
                

                print("Moving")


            #Otherwise there is a wall in front of the laser
            else:


                #Time that the robot has stopped, used to add a time delay when detecting blue
                self.stoppingTime=time.time()

                #Assign stop to true
                self.stop = True
              
                #stop the robot
                self.stopMethod()
                print("Stopping")


                #Checks if the left distance is greater than the right and turns in this direction
                if self.leftDistance > 0.5 and self.leftDistance > self.rightDistance:  
            
                    print("Turning Left")
                    self.turnLeftOrRight(45)
                        

                #Checks if the right distance is greater than the left and turns in this direction
                elif self.rightDistance > 0.5 and self.rightDistance > self.leftDistance:
                
                    print("Turning Right")
                    self.turnLeftOrRight(-45)

    #Robot image function
    def image_callback(self, msg):

        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, d = image.shape

        #Assigns the colour range for green
        lower_green = numpy.array([40,40,40])
        upper_green = numpy.array([70,255,255])

        #Assigns the colour range for yellow
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])

        #Assigns the colour range for blue
        lower_blue = numpy.array([80,50,50])
        upper_blue = numpy.array([130,255,255])

        #Assigns the colour range for red
        lower_red = numpy.array([0,50,50])
        upper_red = numpy.array([20,255,255])

        #Detects the yellow mask
        yellowMask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        #Sets the top and bottom range
        yellow_search_top = 0 
        yellow_search_bot = 1*h/4
        yellowMask[0:yellow_search_top, 0:w] = 0
        yellowMask[yellow_search_bot:h, 0:w] = 0
        yelowM = cv2.moments(yellowMask)
        if yelowM['m00'] > 0: #If in range of yellow
            cx = int(yelowM['m10']/yelowM['m00'])
            cy = int(yelowM['m01']/yelowM['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            
                
        #Detects the blue mask
        blueMask = cv2.inRange(hsv, lower_blue, upper_blue)

        #Sets the top and bottom range
        blue_search_top = 1*h/4
        blue_search_bot = 3*h/4
        blueMask[0:blue_search_top, 0:w] = 0
        blueMask[blue_search_bot:h, 0:w] = 0

        blueM = cv2.moments(blueMask)
        if blueM['m00'] > 0 and  self.stop == False: #If in range of blue and stop is set to false move forwards
            
            cx = int(blueM['m10']/blueM['m00'])
            cy = int(blueM['m01']/blueM['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2

            #If theres nothing in the way move forwards
            if self.rightDistanceRangeToLeft > 0.5:
          
                self.moveForward(err,100)

        #Detects the green mask
        greenMask = cv2.inRange(hsv,lower_green,upper_green)

        #Sets the top and bottom range
        green_search_top = 3*h/4
        green_search_bot = h
        greenMask[0:green_search_top,0:w] = 0
        greenMask[green_search_bot:h,0:w] = 0
        greenM = cv2.moments(greenMask)
        if greenM['m00'] > 0: #If in range of green move forwards
            cx = int(greenM['m10']/greenM['m00'])
            cy = int(greenM['m01']/greenM['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2

            #If theres nothing in the way move forwards
            if self.rightDistanceRangeToLeft > 0.5:

                self.moveForward(err,100)


        #Detects the red mask
        redMask = cv2.inRange(hsv, lower_red, upper_red)
        red_search_top = 3*h/4
        red_search_bot = h
        redMask[0:red_search_top, 0:w] = 0
        redMask[red_search_bot:h, 0:w] = 0
        redM = cv2.moments(redMask)
        if redM['m00'] > 0: #If in range of red
            cx = int(redM['m10']/redM['m00'])
            cy = int(redM['m01']/redM['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 255, 0), -1)
           
            #Stop turn around and turn the laser off to avoid getting stuck
            self.stopMethod()   
            self.turnLeftOrRight(180)    
            self.laserOn = False   
            self.laserOffTime = time.time()     
        
        cv2.imshow("window", image)
        cv2.waitKey(3)
    
    #Stop function which sets the linear and angular velocity to 0
    def stopMethod(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)


    #Move forward function which uses the error divded by a value to adjust the angular velocity
    #The linear x velocity is set to 0.2 making the robot move forward
    def moveForward(self,error,divisibleValue):


        self.twist.angular.z = -float(error) / divisibleValue
        self.twist.linear.x = 0.2
        self.cmd_vel_pub.publish(self.twist)



    #Function that turns the robot based on the angle passed in
    def turnLeftOrRight(self,angle):
            
        #Sets the linear x velocity to 0 and the angular z to the angle passed in 
        self.twist.linear.x = 0
        self.twist.angular.z = math.radians(angle); #45 deg/s in radians/s
        self.cmd_vel_pub.publish(self.twist)
      

       
              

rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()