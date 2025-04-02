import rclpy
from rclpy.node import Node
import numpy as np
import math
#from lidar_data import lidar_data
from std_msgs.msg import Float32MultiArray  
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#global a
fov_l=20
fov_r=160
rb=0.3
vel_max=2
alpha=0.1
k=0.1

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')

        self.subscription=self.create_subscription(LaserScan,'scan',self.lidar_callback,10)
        self.subscription

        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)

    def lidar_callback(self,msg):
        angles,length=self.angle_assignment(msg)
        smoothened_array=self.preprocess(msg,angles)
        new_array = self.chopped_array(smoothened_array,length)
        index ,sign= self.parity(new_array)
        corrected_angle = self.correction(new_array, index,sign)
        phi1, phi2, phi_index = self.gap_array(corrected_angle)
        phi_gap = self.heading_angle(phi1, phi2, new_array, phi_index)
        turn_vel = self.turn(phi_gap)
        vel = self.throttle(phi_gap)

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.steering_angle = phi_gap*(np.pi/180)  
        ackermann_msg.drive.steering_angle_velocity = turn_vel  
        ackermann_msg.drive.speed = vel  
        self.ackermann_publisher.publish(ackermann_msg)

        self.get_logger().info(f"Phi Gap: {phi_gap}, Turn Vel: {turn_vel}, Velocity: {vel}")
            
        
    def angle_assignment(self,msg,angle_min=0,angle_max=4.71239):
        length = len(np.array(msg.ranges))
        angles = np.linspace(angle_min,angle_max,len(np.array(msg.ranges))) * 180 / np.pi
        return angles,length

    def preprocess(self, msg,angles):
        ranges=np.array(msg.ranges)
        smoothened_array=[]
        for i in range(0, len(ranges)):
            smoothened_array.append((ranges[i],angles[i]))
        return smoothened_array

    def chopped_array(self,l,length):
        new_list = []
        for i in range(0, length):
            if l[i][1] > fov_l and l[i][1] < fov_r:
                new_list.append(l[i])

        new_array = np.array(new_list)
        return new_array

    def parity(self, new_array):
        # global l
        # l=lidar_data()
        # Used to extract the distance from the tuple and calculate the difference
        diff = np.diff([t[0] for t in new_array])
        threshold = 0.02 # Used to set a particular threshold for discontinuity identification
        index = np.where(np.abs(diff) > threshold)[0]
        if diff[index[0]]<0:
            sign=-1
        else:
            sign=1
        # index1=np.where(-diff>threshold)[0]
        print(diff)
        print(index)
        # print (index1)
        #print(new_array[index[1]][1])
        # print(index1+1)
        return index,sign
    
    def correction(self, new_array, index,sign):
        corrected_angle = []
        for i in index:
            corr = (rb/new_array[i][0])*180/3.14
            corrected_angle.append(new_array[i][1]+(sign*corr))
            sign=sign*-1


        print(corrected_angle)
        return corrected_angle
    
    def gap_array(self, corrected_angle):
        #ga = np.concatenate(([fov_l], corrected_angle, [fov_r]))
        ga=corrected_angle
        print(ga)
        final_ga = np.diff(ga)
        phi1_index = np.argmax(final_ga)
        phi2_index = phi1_index+1
        phi1 = ga[phi1_index]
        phi2 = ga[phi2_index]
        print(phi1)
        print(phi2)
        print(final_ga)
        return phi1, phi2, phi1_index
    
    def heading_angle(self, phi1, phi2, new_array, phi_index):
        d1 = new_array[phi_index][0]
        d2 = new_array[phi_index+1][0]
        print(d1, d2)
        denominator = np.sqrt(d1*d1+d2*d2+2*d1*d2*np.cos(np.radians(phi2-phi1)))
        phi_gap = np.degrees(
            np.arccos((d1+(d2*(np.cos(np.radians(phi2-phi1)))))/denominator))+phi1

        return phi_gap
    
    def throttle(self, phi_gap):
        vel = vel_max*(1-math.exp(-1*alpha*phi_gap))
        return vel
    
    def turn(self, phi_gap):
        turn_vel = k*phi_gap
        return turn_vel
        

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
