import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
from PIL import Image
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher_node')

        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile
        )

        self.yaml_file = self.declare_parameter('yaml_file', '').value
        self.occupancy_grid = self.load_map()
        if self.occupancy_grid is None:
            self.get_logger().fatal('Map loading failed. Shutting down node.')
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0, self.publish_map)

    def load_map(self):
        try:
            pkg_path = get_package_share_directory('map_publisher')
            maps_path = os.path.join(pkg_path, 'maps')

            yaml_file = os.path.expanduser(str(self.yaml_file)).strip()
            if not yaml_file:
                yaml_file = os.path.join(maps_path, 'my_map.yaml')
            elif not os.path.isabs(yaml_file):
                yaml_file = os.path.abspath(yaml_file)

            with open(yaml_file, 'r') as f:
                map_metadata = yaml.safe_load(f)

            image_name = map_metadata['image']
            pgm_file = image_name if os.path.isabs(image_name) else os.path.join(
                os.path.dirname(yaml_file), image_name
            )

            if not os.path.exists(pgm_file):
                self.get_logger().error(f'PGM file not found: {pgm_file}')
                return None

            image = Image.open(pgm_file).convert('L')
            image_array = np.array(image)

        except Exception as e:
            self.get_logger().error(f'Failed to load map files: {e}')
            return None

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'

        grid.info.resolution = float(map_metadata['resolution'])
        grid.info.width = image_array.shape[1]
        grid.info.height = image_array.shape[0]

        origin = map_metadata['origin']
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.z = np.sin(origin[2] / 2.0)
        grid.info.origin.orientation.w = np.cos(origin[2] / 2.0)

        occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
        free_thresh = map_metadata.get('free_thresh', 0.25)

        image_array = np.flipud(image_array)
        data = []

        for pixel in image_array.flatten():
            normalized = pixel / 255.0

            if normalized < occupied_thresh:
                data.append(100)
            elif normalized > free_thresh:
                data.append(0)
            else:
                data.append(-1)

        grid.data = data

        self.get_logger().info('✅ Map loaded successfully')
        self.get_logger().info(f'   Size: {grid.info.width} x {grid.info.height}')
        self.get_logger().info(f'   Resolution: {grid.info.resolution}')

        return grid

    def publish_map(self):
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
