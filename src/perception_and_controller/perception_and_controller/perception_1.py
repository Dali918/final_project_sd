import rclpy # Import the ROS 2 client library for Python
from rclpy.node import Node # Import the Node class for creating ROS 2 nodes
from sensor_msgs.msg import Image,PointCloud2
from geometry_msgs.msg import PoseStamped,Pose


import cv2
from cv_bridge import CvBridge

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import torch
from Perception.model.unet import UNet
from Perception.evaluate import evaluate

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .rotation_utils import transform_pose

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning) 

#import asyncio ##

device = "cpu"
 
class Perception(Node):
    
 
    def __init__(self,trained_model = "epoch_39.pt"):
        super().__init__('Perception')
        self.image_subscription = self.create_subscription( # Subscribes to camera images topic
            Image,
            '/oakd/rgb/image_raw',
            self.image_callback,
            10)
        self.pose_publisher = self.create_publisher( # Publishes target waypoints to ./pose_msg topic
            PoseStamped,
            '/pose_msg', 
            10                 
        )

        # tf buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.br = CvBridge()

        save_dict = torch.load(trained_model, map_location=device)
        self.model =  UNet()
        self.model.load_state_dict(save_dict["model"])
        self.model.eval() # Uses model to evaluate camera image from car and returns labelled image as 'pred'
        self.pose_msg = PoseStamped()
        self.std_dev = 0
        

    def calculate_path(self,current_frame,pred):
        # Calculates new y waypoint (middle) for vehicle based on predicted image (pred)

        'Method 1'
        (h,w) = pred.shape # height and width of labelled image after being run through model
        
        watch_pixel = 160
        left = pred[:,:w//2] # all pixels on left side of pred image are saved as 'left' array
        right = pred[:,w//2:] # all pixels on right side of pred image are saved as 'right' array
        min_left = 0 # minimum left point or leftmost column(y) of image
        max_right = w - 1 # maximum right point or rightmost column(y) of image

        (l_h,l_w) = left.shape
        (r_h,r_w) = right.shape

        # Left loop: Loops through left array representing the left side of the predicted image. 
        # A value of 0 indicates the color black/no label in the predicted image. A nonzero value indicates
        # the color white/presence of a label in the predicted image (most likely a lane or some obstruction).
        for i in range(0, l_h): 
            for j in range(l_w):
                if(left[i,j] != 0):# presence of a lane or obstruction
                    if j > min_left: # if a label is found and is greater than the current leftmost label/lane in the image, its corresponding column becomes the new leftmost y value
                        min_left = j          
        left_middle = min_left

        # Right Loop: Follows the same conventions as the left loop for the right side of the predicted image. 
        for k in range(0, r_h): 
            for l in range(r_w):
                if(right[k,l] != 0):
                    if l < max_right:
                        max_right = l
                    	#sum_horizontal += right[k,l]*l
                    	#sum_r += 1
        right_middle = max_right + (w//2) # max_right value (l) would still be calculated from 0 to r_w so it's added to the middle value w//2 

        middle = (right_middle + left_middle)/2 # Calculated middle value (*becomes waypoint for controller algorithm*)
        

        current_frame = cv2.resize(current_frame,(w,h),cv2.INTER_AREA)
        #row, column, channel
        
        # Labels on camera image
 
        current_frame[watch_pixel,:,0] = 250 #B
        current_frame[watch_pixel,:,1] = 200 #G
        current_frame[watch_pixel,:,2] = 50 #R

	    # Highlights lanes from perception
        current_frame[pred.astype(bool),1:] = 75 
        
        # left_middle = blue, right_middle = green, middle = red
        current_frame[:,int(left_middle),0] = 255 #B
        current_frame[:,int(right_middle),1] = 255 #G
        current_frame[:,int(middle),2] = 255 #R

	    # gray line always in middle of camera frame
        current_frame[:,int(w/2),0] = 100 #255
        current_frame[:,int(w/2),1] = 100
        current_frame[:,int(w/2),2] = 100
        
        #pred[watch_pixel,:] = 250 #B
        #pred[watch_pixel,:] = 200 #G
        #pred[watch_pixel,:] = 50 #R
        
        # Labels on predicted camera image (black and white image)

        pred[:,int(left_middle)] = 255 #B
        pred[:,int(right_middle)] = 255 #G
        pred[:,int(middle)] = 255 #R
        
        pred[:,int(w/2)] = 100 #255
        #pred[:,int(w/2)] = 100
        #pred[:,int(w/2)] = 100

  
        cv2.imshow("camera", current_frame)   
        cv2.imshow("prediction", pred)  
        cv2.waitKey(1)

        # Transforming middle waypoint found from base link (car coordinate system) to world link (raceway coordinate system)
        self.pose_msg.header.frame_id = "base_link"
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.pose.position.x = 5.0
        self.pose_msg.pose.position.y = (w//2 - middle)/175 # (w//2 - middle) is how much correction is done between middle point of lanes and middle of image. In original algorithm, y = (w//2 - middle)/10
        self.pose_msg.pose.position.z = 0.0
        self.pose_msg.pose.orientation.w = 1.0

        # Transform waypoint to world link coordinate system and publish to '/pose_msg' topic
        self.transform_and_publish_pose(self.pose_msg)



    def image_callback(self,msg : Image):
        # Called when camera image from car is received
        current_frame = self.br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        pred = evaluate(self.model,current_frame) # Run current frame from camera through model
        self.calculate_path(current_frame,pred) # Calculate path (middle waypoint) function
    
    
    def transform_and_publish_pose(self,pose_msg : PoseStamped):
        try: # Transform from base link frame to world frame
            #print (pose_msg.pose) ###
            t = self.tf_buffer.lookup_transform( # asyncio.run(self.tf_buffer.lookup_transform_async
                "world", 
                pose_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            pose_msg.pose = transform_pose(pose_msg.pose, t)
            ##print(pose_msg.pose) ###
            pose_msg.header.frame_id = "world" 
            

        except TransformException as ex:
            self.get_logger().info( 
                f"Could not transform {pose_msg.header.frame_id} to base_link: {ex}"
            )
            return
        
        self.pose_publisher.publish(self.pose_msg)

        

 
def main(args=None):
 
    rclpy.init(args=args)

    perception = Perception()
 

    rclpy.spin(perception)
 

    perception.destroy_node()
 

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()