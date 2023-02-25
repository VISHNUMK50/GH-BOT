#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist,Vector3
from sensor_msgs.msg import LaserScan
import time
from math import radians
# from pid_tune.msg import PidTune

class controller:
    def __init__(self):
        """Initializes the parameters for ROS.
        """
        # Creating a velocity publisher which publishes to the the topic '/cmd_vel' topic.
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
        # rospy.Subscriber('/pid_tuning', PidTune, self.set_pid_value )

        # This is the Linear velocity value.
        self.lin_speed = 0
        # This is the Angular velocity value.
        self.ang_speed = 0
        self.regions = {
            'bright': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'bleft': 0,
        }
        self.t1 = 0
        self.t0 = 0
        self.error = 0
        self.kp = 2
        self.pid_output = 0
        self.prev_error = 0
        self.kd = 1
        self.ki = 0
        self.sum_err = 0
    # def set_pid_value(self,data):
	# 	self.kp = data.Kp 
	# 	self.kp_y = data.Kp 
	# 	self.kp_z = data.Kp
		
	# 	self.kd =  data.Kd
	# 	self.kd_y =  data.Kd
	# 	self.kd_z = data.Kd
		
	# 	self.ki = data.Ki
	# 	self.ki_y = data.Ki
	# 	self.ki_z = data.Ki

    def move(self,lin_speed,ang_speed):

        """Function to publish velocities.
        Args:
            lin_speed (float): The desired linear velocity.
            ang_speed (float): The desired Angular velocity.
        """
        while not rospy.is_shutdown():
            # Creating a Twist message.
            vel = Twist()
            # Set Linear and Angular velocities.
            vel.linear = Vector3(lin_speed, 0, 0)
            vel.angular = Vector3(0, 0, ang_speed)
            for _ in range(2):
                self.pub.publish(vel)
                rospy.sleep(.1)
            break

    def rotate(self, angle,speed):
        """Function which rotates to the desired angle.
        Args:
            angle (float): Desired angle.
        """
        # Creating a Twist message.
        vel = Twist()
        # Converting the angle to radians.
        angle = radians(angle)
        angle_travelled = 0.0

        # Stop for safety reasons.
        self.stop()
        vel.linear = Vector3(0, 0, 0)
        vel.angular = Vector3(0, 0, speed)
        # Start SIM time.
        ts = rospy.Time.now().to_sec()

        # Traverse said angle.
        while angle_travelled < angle:
            # Stop SIM time.
            te = rospy.Time.now().to_sec()
            # Calculating the angle travelled.
            angle_travelled = speed * (te-ts)
            # Publishing the message.
            for i in range(2):
                self.pub.publish(vel)
                rospy.sleep(.1)

        # Stop for safety reasons.
        self.stop()


    def pid_cntrl(self,c_dist,relative_dist):
        self.prev_error = self.error
        self.error = c_dist-relative_dist
        tt=self.t1-self.t0
        self.t0 = rospy.Time.now().to_sec()
        self.sum_err = self.sum_err + self.error
        self.pid_output = self.kp * self.error + (self.kd * ((self.error - self.prev_error)/tt)) + (self.ki * self.sum_err * tt) 

   


    def stop(self):
        """Function which stops the robot.
        """
        # Creating a Twist message.
        vel = Twist()

        # Set Linear and Angular velocities.
        vel.linear = Vector3(0, 0, 0)
        vel.angular = Vector3(0, 0, 0)
        # Publishing the message.
        for _ in range(2):
            self.pub.publish(vel)
            rospy.sleep(.1)

    def laser_callback(self,msg):
        global regions, range_max
        range_max=msg.range_max 
        self.regions = {
            'bright': min(min(msg.ranges[0:143]), range_max),   # msg.ranges[0:144] ,
            'fright': min(min(msg.ranges[144:288]), range_max), # msg.ranges[144:288] ,
            'front': min(min(msg.ranges[289:432]), range_max),  # msg.ranges[288:432] ,
            'fleft': min(min(msg.ranges[433:576]), range_max),  # msg.ranges[432:576] ,
            'bleft': min(min(msg.ranges[577:720]), range_max),  # msg.ranges[576:720] ,
        }
#    print(regions)

def ebot_nav():
    """Navigating through racks 
    """
    # Accessing the global variable ctrl.
    global ctrl, rate,regions,pid_output,r,c,tot_rack,RT,NRT,rack
    
    # print("im navigating")
    while not rospy.is_shutdown():
        #print(ctrl.regions)
        reg=ctrl.regions
        pid_out=ctrl.pid_output
        #
        # Your algorithm to navigate
        #
        # print("im checking regions")
        # print(reg)
# at home --------------- {'bright': 2.545, 'fright': 1.050, 'front': 1.907, 'fleft': 1.042, 'bleft': 3.103}
# at the rack start ----- {'bright': 0.565, 'fright': 0.717, 'front': 1.898, 'fleft': 0.731, 'bleft': 0.574}
# at the rack end ------- {'bright': 2.986, 'fright': 0.981, 'front': 0.914, 'fleft': 0.993, 'bleft': 2.625}
# after turn ------------ {'bright': 1.574, 'fright': 1.134, 'front': 1.127, 'fleft': 1.709, 'bleft': 1.079}
# jst befor hit at 6 ---- {'bright': 1.149, 'fright': 3.711, 'front': 0.957, 'fleft': 0.601, 'bleft': 0.570}

       
        
        '''
        if (0.50 < reg['bleft'] < 0.52 and reg['front'] >= 0.6):
            print("im moving")
            print(reg)
            ctrl.move(1,0)


            #stick to the left rack
        elif(reg['bleft'] >= 0.52 and reg['front'] >= 0.6 ):
            print("im moving left")
            print(reg)
            ctrl.move(1,0.2)    #moving left

        elif(reg['bleft'] <= 0.50 and reg['front'] >= 0.6 ):
            print("im moving right")
            print(reg)
            ctrl.move(1,-0.2)   #im moving right
        
            #detect rack end 
        elif(reg['bleft'] >= 1 and reg['front'] >= 0.6 ):
            print("im moving right")
            print(reg)
            ctrl.move(1,-0.5)   #im moving right
        '''
        '''
        # if(reg['bleft'] >= 1.8 and reg['bright'] >= 1.8 and reg['front'] >= 1.5): # Entering the rack
        #     print("im at HOME ")
        #     print("im entering the rack")
        #     ctrl.move(1,0)
        # elif(reg['bleft'] <= 1.2 and  reg['front'] >= 1): # Navigating through the rack
        #     print("im in the rack ")
        #     ctrl.t1 = rospy.Time.now().to_sec()
        #     ctrl.pid_cntrl(reg['bleft'])
        #     print("working on pid, out = ",pid_out)
        #     ctrl.move(1,pid_out)
        # elif(reg['bleft'] >= 1 and reg['fleft'] >= 1  ): # Reached the rack end
        #     print("im at the end ")
        #     ctrl.move(0.4,(1+pid_out))
'''     
        # print('c=',c)
        # print('r=',r)
        if(r!=tot_rack):
            if(reg['bleft'] >= 1.5 and reg['bright'] >= 1.8 and reg['front'] >= 1.5): # Entering the rack
                # print('searching for rack ==== moving fwrd')
                ctrl.move(1,0)
            elif(reg['bleft'] <= 1 ): # Entering the rack

                # print('rack on the left ==== folowing rack')
                RT = rospy.Time.now().to_sec()
                # print('RT=',RT)
                rack=True
                ctrl.t1 = rospy.Time.now().to_sec()
                ctrl.pid_cntrl(reg['bleft'],0.56)
                # print("working on pid, out = ",pid_out)
                ctrl.move(1,pid_out)




            elif(reg['bleft'] >= 1.1 ): # rack end
                NRT = rospy.Time.now().to_sec()
                # print('NRT=',NRT)
                if(rack == True and NRT>RT):
                    c+=1
                    rack = False

                    if((c%4 != 0 and c/4 !=1) or (c%4 == 0 and c/4 !=1 )): # im at an end
                        # print('turn left')    
                        if (c%4 ==0 ):
                            r+=1 
                        for i in range(18):
                        # if(reg['bleft'] >= .5 ):
                            ctrl.move(.25,.9)
                            # print("i=",i)
                        
                    elif(c%4 == 0 and c/4 ==1 and r == 1): # im at 4th cornner
                        # print('at 4th corner , moving straight')
                        if (c%4 ==0 ):
                            r+=1
                        ctrl.t1 = rospy.Time.now().to_sec()
                        ctrl.pid_cntrl(reg['bright'],0.5)
                        #print("working on pid, out = ",pid_out)
                        ctrl.move(1,pid_out)
                
            # else:
            #     ctrl.t1 = rospy.Time.now().to_sec()
            #     ctrl.pid_cntrl(reg['bright'],0.6)
            #     #print("working on pid, out = ",pid_out)
            #     ctrl.move(1,pid_out)
        # print('i dont know what to do')
        if(r==tot_rack):
            print('completed')

        
        # print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()



def main():
    """Main function.
    """
    global ctrl,rate,regions,pid_output,r,c,tot_rack,RT,NRT,rack
    tot_rack=2
    r=0
    c=0
    RT=float('inf')
    rack=False
    # Initializing node.
    rospy.init_node("ebot_controller")
    rate = rospy.Rate(10) 
    ctrl = controller()
    rospy.sleep(1)

    ebot_nav()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
