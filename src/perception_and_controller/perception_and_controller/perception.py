import rclpy # Import the ROS 2 client library for Python
from rclpy.node import Node # Import the Node class for creating ROS 2 nodes
from sensor_msgs.msg import Image,PointCloud2
from geometry_msgs.msg import PoseStamped,Pose


import cv2
from cv_bridge import CvBridge

import numpy as np
import os
import sensor_msgs_py.point_cloud2 as pc2
import torch
from Perception.model.unet import UNet
from Perception.evaluate import evaluate

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .rotation_utils import transform_pose

from ament_index_python.packages import get_package_share_directory


import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning) 

device = "cpu"
 
class Perception(Node):
    
 
    # Get the package's share directory
    package_name = 'perception_and_controller'
    package_share_dir = get_package_share_directory(package_name)

    # Path to the model file in the models directory
    TRAINED_MODEL_PATH = os.path.join(package_share_dir, 'models', 'epoch_39.pt')

    def __init__(self, trained_model=TRAINED_MODEL_PATH):
        super().__init__('Perception')
        self.image_subscription = self.create_subscription(
            Image,
            '/oakd/rgb/image_raw',
            self.image_callback,
            10)
        self.pose_publisher = self.create_publisher(
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
        self.model.eval()
        self.pose_msg = PoseStamped()

    def find_lane_points(self, img_half, is_left=True, start_h=0):
        """Find lane points in image half (left or right side)"""
        (h, w) = img_half.shape
        
        if is_left:
            min_point = 0
            for i in range(start_h, h):
                for j in range(w):
                    if img_half[i,j] != 0 and j > min_point:
                        min_point = j
            return min_point
        else:
            max_point = w - 1
            for i in range(start_h, h):
                for j in range(w):
                    if img_half[i,j] != 0 and j < max_point:
                        max_point = j
            return max_point


    def calculate_path(self,current_frame,pred):
        
        # Calculates new y waypoint (middle) for vehicle based on predicted image (pred)

        'Method 1'
        (h,w) = pred.shape # height and width of labelled image after being run through model
        
        if not hasattr(self, 'prev_left_middle'):
            self.prev_left_middle = 0

        watch_pixel = 200
        left = pred[:,:w//2] # all pixels on left side of pred image are saved as 'left' array
        right = pred[:,w//2:] # all pixels on right side of pred image are saved as 'right' array
        min_left = 0 # minimum left point or leftmost column(y) of image
        start_h = 0    # which height to use for calculating the min 

        # First try with full height
        left_middle = self.find_lane_points(left,start_h=watch_pixel,is_left=True)
        right_middle = self.find_lane_points(right,start_h=watch_pixel,is_left=False) + w//2

        # Check lane distance
        LANE_DISTANCE_THRESHOLD = w//95   # Adjust based on testing
        lane_distance = right_middle - left_middle

        
       # If lanes too close or left lane lost, try bottom half
        if lane_distance < LANE_DISTANCE_THRESHOLD:
            start_h = watch_pixel
            left_middle = self.find_lane_points(left, is_left=True, start_h=start_h)
            right_middle = self.find_lane_points(right, is_left=False, start_h=start_h) + w//2   

        #TODO: might have to get rid of turn threshold with new find lane updates
        TURN_THRESHOLD = 30    
        delta_left = min_left - self.prev_left_middle  
        is_sharp_left = delta_left > TURN_THRESHOLD

        if is_sharp_left:
            left_middle = 0  # Force to leftmost position

        self.prev_left_middle = left_middle

        middle = (right_middle + left_middle)/2 # Calculated middle value (*becomes waypoint for controller algorithm*)

        current_frame = cv2.resize(current_frame,(w,h),cv2.INTER_AREA)
        #row, column, channel
        
        # Labels on camera image
 
        current_frame[watch_pixel,:,0] = 250 #B
        current_frame[watch_pixel,:,1] = 200 #G
        current_frame[watch_pixel,:,2] = 50 #R

        # current image height
        current_frame[int(start_h),:,0] = 250 #B
        current_frame[int(start_h),:,1] = 100 #G
        current_frame[int(start_h),:,2] = 50 #R

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
        #pred[:,int(w/2)] = 100        def calculate_path(self, current_frame, pred):
  
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
        
        current_frame = self.br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        pred = evaluate(self.model,current_frame)
        self.calculate_path(current_frame,pred)
    
    
    def transform_and_publish_pose(self,pose_msg : PoseStamped):
        try:
            t = self.tf_buffer.lookup_transform(
                "world",
                pose_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            pose_msg.pose = transform_pose(pose_msg.pose, t)
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
