import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class OccupancyGridGenerator(Node):
    def __init__(self):
        super().__init__('occupancy_grid_generator')
        
        # Parameters for the occupancy grid
        self.grid_resolution = 0.1  # meters per cell
        self.grid_width = 20.0  # meters
        self.grid_height = 20.0  # meters
        
        # Calculate grid size in cells
        self.width_cells = int(self.grid_width / self.grid_resolution)
        self.height_cells = int(self.grid_height / self.grid_resolution)
        
        # Initialize tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to pointcloud data
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'oakd/rgb/depth/points',  # Adjust topic name as needed
            self.pointcloud_callback,
            10)
            
        # Publisher for occupancy grid
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            'occupancy_grid',
            10)
            
        # Initialize empty grid
        self.occupancy_grid = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid cell indices"""
        grid_x = int((x + self.grid_width/2) / self.grid_resolution)
        grid_y = int((y + self.grid_height/2) / self.grid_resolution)
        return grid_x, grid_y
        
    def pointcloud_callback(self, msg):
        # Reset grid
        self.occupancy_grid.fill(0)
        
        try:
            # Get transform from camera frame to base frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                msg.header.stamp)
                
            # Process point cloud
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                # Transform point to base frame
                # Note: This is simplified - you should use proper transform
                x = point[0]
                y = point[1]
                z = point[2]
                
                # Skip points too high or too low
                if z > 1.0 or z < 0.1:
                    continue
                    
                # Convert to grid coordinates
                grid_x, grid_y = self.world_to_grid(x, y)
                
                # Check if point is within grid bounds
                if (0 <= grid_x < self.width_cells and 
                    0 <= grid_y < self.height_cells):
                    # Mark as occupied (100 for occupied, 0 for free)
                    self.occupancy_grid[grid_y, grid_x] = 100
                    
            # Create and publish OccupancyGrid message
            grid_msg = OccupancyGrid()
            grid_msg.header = msg.header
            grid_msg.header.frame_id = 'base_link'  # Or your preferred frame
            grid_msg.info.resolution = self.grid_resolution
            grid_msg.info.width = self.width_cells
            grid_msg.info.height = self.height_cells
            grid_msg.info.origin.position.x = -self.grid_width/2
            grid_msg.info.origin.position.y = -self.grid_height/2
            
            # Flatten grid to 1D array
            grid_msg.data = self.occupancy_grid.flatten().tolist()
            
            self.grid_pub.publish(grid_msg)
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform pointcloud: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()