import rospy
import time
from sensor_msgs.msg import Imu
import roboclaw

class RoboClaw:
    """ 
    used to control the roboclaw
    """
    def __init__(self, port = '/dev/ttyACM1'):
        # Roboclaw parameters
        self.m1Duty = 0
        self.m2Duty = 0
        #largest absolute value that the motors can be set to
        self.dutyMax = 5000
        
        self.address = 0x80
        roboclaw.Open(port,115200)

        
    def writeM1M2(self,m1Duty,m2Duty):
        if abs(m1Duty) > self.dutyMax:
            self.m1Duty = self.dutyMax * cmp(m1Duty, 0)
        else:
            self.m1Duty = m1Duty
            
        if abs(m2Duty) > self.dutyMax:
            self.m2Duty = self.dutyMax * cmp(m2Duty, 0)
        else:
            self.m2Duty = m2Duty
         
        #print self.m1Duty,self.m2Duty
        roboclaw.DutyAccelM1(self.address,5000,self.m1Duty)
        roboclaw.DutyAccelM2(self.address,5000,self.m2Duty)
        
class DataRecorder:
    """
    Records data from IMU's
    """
    def __init__(self):
        # Roboclaw
        self.myRoboclaw = RoboClaw()
        
        # Process control variables
        self.test_in_progress = False           
        self.m1v_history = []           # array of tuples of (time, velocity)
        self.m2v_history = []           # array of tuples of (time, velocity)
        self.start_time = 0
        
        # ROS subscribers
        rospy.Subscriber("/imu1", Imu, self._imu1Callback)
        rospy.Subscriber("/imu2", Imu, self._imu2Callback)
        time.sleep(1)                
    
    def _imu1Callback(self,data):
        if self.test_in_progress:
            time_now = time.time() - self.start_time
            current_m1v = data.angular_velocity.z
            self.m1v_history.append( (time_now, current_m1v) )
        
    def _imu2Callback(self,data):
        if self.test_in_progress:
            time_now = time.time() - self.start_time
            current_m2v = data.angular_velocity.z
            self.m2v_history.append( (time_now, current_m2v) )
            
    def record_results(self, file_name):
        f = open(file_name + '_m1', 'w')
        for datum in self.m1v_history:
            f.write(datum[0] + ',' + datum[1] + '\n')
            
        f = open(file_name + '_m2', 'w')
        for datum in self.m2v_history:
            f.write(datum[0] + ',' + datum[1] + '\n')            

    def loop(self):
        self.myRoboclaw.writeM1M2(0, 0)
        
        while True:
            ## ASK FOR OPTIONS
            time.sleep(.05)
            run_another = raw_input("Would you like to run a test (y/n)?")
            if run_another == 'n':
                return
                
            file_name = raw_input("Filename?\n")
            pwm = raw_input("What speed for motors?\n")
            time_to_run = raw_input("How long to run?\n")
            self.start_time = time.time()
            
            ## RUN TEST
            self.myRoboclaw.writeM1M2(int(pwm), int(pwm))
            while time.time() - self.start_time < int(time_to_run):
                time.sleep(.05)
                
            ## STOP TEST
            print "Done running!"
            self.myRoboclaw.writeM1M2(0, 0)
            
            ## WRITE TEST RESULTS
            print "Recording results!"
            self.record_results(file_name)
            
            ## REFRESH MEMORY
            self.m1v_history = []
            self.m2v_history = []