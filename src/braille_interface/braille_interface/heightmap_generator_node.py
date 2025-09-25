import rclpy
from rclpy.node import Node

from nvblox_msgs.msg import Mesh
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2
import math

DOT_ROWS = 36
DOT_COLS = 80

class BrailleHeightmapNode(Node):
    def __init__(self):
        super().__init__('braille_heightmap_node')

        self.declare_parameter('source_type', 'occupancy')
        self.declare_parameter('heightmap_size', 5.0)
        self.declare_parameter('invert_colors', False)

        self.source_type = self.get_parameter('source_type').get_parameter_value().string_value.lower()
        self.size = self.get_parameter('heightmap_size').get_parameter_value().double_value
        self.invert_colors = self.get_parameter('invert_colors').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.latest_pose = None
        self.trail = []

        self.create_subscription(PoseStamped, '/visual_slam/tracking/vo_pose', self.pose_callback, 10)

        if self.source_type == 'mesh':
            self.create_subscription(Mesh, '/nvblox_node/mesh', self.mesh_callback, 10)
            self.get_logger().info('Using mesh as source.')
        elif self.source_type == 'occupancy':
            self.create_subscription(OccupancyGrid, '/nvblox_node/static_occupancy_grid', self.occupancy_callback, 10)
            self.get_logger().info('Using occupancy grid as source.')
        else:
            self.get_logger().warn(f"Invalid source_type: {self.source_type}. Valid options: 'mesh', 'occupancy'.")

        self.image_pub = self.create_publisher(Image, '/braille/heightmap', 10)
        self.debug_pub = self.create_publisher(Image, '/braille/heightmap_generator_debug_viz', 10)

        self.get_logger().info('BrailleHeightmapNode initialized.')

    def pose_callback(self, msg):
        self.latest_pose = msg.pose
        self.trail.append((msg.pose.position.x, msg.pose.position.y))
        if len(self.trail) > 100:
            self.trail.pop(0)

    def mesh_callback(self, msg):
        all_points = [ (v.x, v.y, v.z) for block in msg.blocks if block.vertices for v in block.vertices ]
        if not all_points:
            return

        vertices_np = np.array(all_points)
        center = (self.latest_pose.position.x, self.latest_pose.position.y) if self.latest_pose else None

        yaw = 0.0
        if self.latest_pose:
            q = self.latest_pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        heightmap = self.generate_heightmap(vertices_np, grid_size=(60, 60), center=center, size=self.size, yaw=yaw)
        normalized = cv2.normalize(heightmap, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.publish_images(normalized, source='mesh')

    def occupancy_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        occ_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        raw_map = np.full_like(occ_data, 128, dtype=np.uint8)

        if self.invert_colors:
            raw_map[occ_data == 0] = 0        # Free → black
            raw_map[occ_data == 100] = 255    # Occupied → white
        else:
            raw_map[occ_data == 0] = 255      # Free → white
            raw_map[occ_data == 100] = 0      # Occupied → black

        braille_map = self.preprocess_and_pad(raw_map, target_shape=(DOT_ROWS, DOT_COLS))
        self.publish_images(braille_map, source='occupancy')

    def publish_images(self, image, source=''):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(img_msg)
            self.get_logger().debug(f"Published heightmap image from source: {source}")
        except Exception as e:
            self.get_logger().error(f"Error publishing heightmap image: {e}")

        try:
            dbg_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if source == 'mesh':
                dbg_img = self.draw_pose_overlay(dbg_img, image.shape)
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg_img, encoding='bgr8')
            dbg_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(dbg_msg)
            self.get_logger().debug(f"Published debug image from source: {source}")
        except Exception as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

    def draw_pose_overlay(self, debug_image, grid_shape):
        if self.latest_pose:
            h, w = grid_shape
            q = self.latest_pose.orientation
            x, y = self.latest_pose.position.x, self.latest_pose.position.y
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

            def world_to_px(wx, wy):
                half_size = self.size / 2.0
                return int((wx - (x - half_size)) / self.size * (w - 1)), int((wy - (y - half_size)) / self.size * (h - 1))

            cx, cy = world_to_px(x, y)
            dx = int(20 * math.cos(yaw))
            dy = int(20 * math.sin(yaw))
            cv2.circle(debug_image, (cx, cy), 6, (0, 0, 255), -1)
            cv2.arrowedLine(debug_image, (cx, cy), (cx + dx, cy + dy), (0, 255, 0), 2)

            for tx, ty in self.trail:
                if abs(tx - x) <= self.size / 2.0 and abs(ty - y) <= self.size / 2.0:
                    px, py = world_to_px(tx, ty)
                    cv2.circle(debug_image, (px, py), 2, (255, 255, 255), -1)
        return debug_image

    def preprocess_and_pad(self, input_img, target_shape):
        h, w = input_img.shape
        target_h, target_w = target_shape
        input_aspect = w / h
        target_aspect = target_w / target_h

        if input_aspect > target_aspect:
            new_w = target_w
            new_h = int(target_w / input_aspect)
        else:
            new_h = target_h
            new_w = int(target_h * input_aspect)

        resized = cv2.resize(input_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        padded = np.full(target_shape, 128, dtype=np.uint8)
        y_offset = (target_h - new_h) // 2
        x_offset = (target_w - new_w) // 2
        padded[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized

        # Draw border frame around resized map
        cv2.rectangle(padded, (x_offset, y_offset), (x_offset + new_w - 1, y_offset + new_h - 1), 255, 1)

        return padded

    @staticmethod
    def generate_heightmap(vertices, grid_size=(60, 60), method='max', center=None, size=5.0, yaw=0.0):
        x_vals, y_vals, z_vals = vertices[:, 0], vertices[:, 1], vertices[:, 2]
        x_center, y_center = center if center else (np.mean(x_vals), np.mean(y_vals))
        x_local = x_vals - x_center
        y_local = y_vals - y_center

        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        x_rot = x_local * cos_yaw - y_local * sin_yaw
        y_rot = x_local * sin_yaw + y_local * cos_yaw - size * 0.4

        half_size = size / 2.0
        mask = (np.abs(x_rot) <= half_size) & (np.abs(y_rot) <= half_size)

        x_pix = (((x_rot[mask] + half_size) / size) * (grid_size[1] - 1)).astype(int)
        y_pix = (((y_rot[mask] + half_size) / size) * (grid_size[0] - 1)).astype(int)

        heightmap = np.full(grid_size, np.nan)
        for i, (x_idx, y_idx) in enumerate(zip(x_pix, y_pix)):
            z = z_vals[mask][i]
            if np.isnan(heightmap[y_idx, x_idx]):
                heightmap[y_idx, x_idx] = z
            else:
                if method == 'max':
                    heightmap[y_idx, x_idx] = max(heightmap[y_idx, x_idx], z)
                elif method == 'mean':
                    heightmap[y_idx, x_idx] = (heightmap[y_idx, x_idx] + z) / 2
        return np.nan_to_num(heightmap, nan=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = BrailleHeightmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
