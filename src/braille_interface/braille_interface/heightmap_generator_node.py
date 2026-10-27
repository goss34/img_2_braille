import rclpy
from rclpy.node import Node

from nvblox_msgs.msg import Mesh
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
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

        # Full accumulated occupancy grid — stored on every incoming message,
        # never cleared, so history is always preserved in memory.
        self.full_occ_raw = None    # np.ndarray (map_h, map_w), dtype uint8
        self.full_occ_meta = None   # dict: resolution, origin_x, origin_y, width, height

        # Last successfully rendered ego-centric crop — used as fallback when
        # pose is unavailable at render time.
        self.latest_crop = None     # np.ndarray (DOT_ROWS, DOT_COLS), dtype uint8

        self.create_subscription(
            PoseStamped, '/visual_slam/tracking/vo_pose', self.pose_callback, 10)

        if self.source_type == 'mesh':
            self.create_subscription(Mesh, '/nvblox_node/mesh', self.mesh_callback, 10)
            self.get_logger().info('Using mesh as source.')
        elif self.source_type == 'occupancy':
            self.create_subscription(
                OccupancyGrid, '/nvblox_node/static_occupancy_grid',
                self.occupancy_callback, 10)
            self.get_logger().info('Using occupancy grid as source.')
        else:
            self.get_logger().warn(
                f"Invalid source_type: '{self.source_type}'. Valid options: 'mesh', 'occupancy'.")

        self.image_pub = self.create_publisher(Image, '/braille/heightmap', 10)
        self.debug_pub = self.create_publisher(Image, '/braille/heightmap_generator_debug_viz', 10)

        # Service: called by braille_pipeline_node when CENTER is pressed in
        # occupancy mode.  Snapshots current pose, crops and publishes to
        # /braille/heightmap so the normal pipeline picks it up.
        self.render_srv = self.create_service(
            Trigger,
            '/braille_heightmap/render_ego_crop',
            self.handle_render_request)

        self.get_logger().info('BrailleHeightmapNode initialized.')

    # ------------------------------------------------------------------
    # Pose
    # ------------------------------------------------------------------

    def pose_callback(self, msg):
        self.latest_pose = msg.pose
        self.trail.append((msg.pose.position.x, msg.pose.position.y))
        if len(self.trail) > 100:
            self.trail.pop(0)

    # ------------------------------------------------------------------
    # Mesh path (unchanged)
    # ------------------------------------------------------------------

    def mesh_callback(self, msg):
        all_points = [
            (v.x, v.y, v.z)
            for block in msg.blocks if block.vertices
            for v in block.vertices
        ]
        if not all_points:
            return

        vertices_np = np.array(all_points)
        center = (
            (self.latest_pose.position.x, self.latest_pose.position.y)
            if self.latest_pose else None
        )

        yaw = 0.0
        if self.latest_pose:
            q = self.latest_pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

        heightmap = self.generate_heightmap(
            vertices_np, grid_size=(60, 60), center=center, size=self.size, yaw=yaw)
        normalized = cv2.normalize(heightmap, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.publish_images(normalized, source='mesh')

    # ------------------------------------------------------------------
    # Occupancy path
    # ------------------------------------------------------------------

    def occupancy_callback(self, msg):
        """
        Store the full incoming occupancy grid every time nvblox publishes one.
        Does NOT render or publish — rendering only happens on demand via
        handle_render_request(), which the pipeline node calls when the
        CENTER button is pressed in occupancy mode.
        """
        width  = msg.info.width
        height = msg.info.height
        res    = msg.info.resolution
        ox     = msg.info.origin.position.x
        oy     = msg.info.origin.position.y

        occ_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Convert to uint8.
        # Default: raised dots = obstacles/walls, lowered = free space.
        #   255 (raised) = occupied or unknown
        #     0 (lowered) = free
        #   128 (mid)    = unobserved (no dot on braille board)
        # Set invert_colors=true to flip raised/lowered if needed.
        raw_map = np.full((height, width), 128, dtype=np.uint8)
        if self.invert_colors:
            raw_map[occ_data == 0]   = 255  # Free     → raised
            raw_map[occ_data == 100] = 0    # Occupied → lowered
            raw_map[occ_data == -1]  = 0    # Unknown  → lowered
        else:
            raw_map[occ_data == 0]   = 0    # Free     → lowered
            raw_map[occ_data == 100] = 255  # Occupied → raised
            raw_map[occ_data == -1]  = 255  # Unknown  → raised

        self.full_occ_raw  = raw_map
        self.full_occ_meta = {
            'resolution': res,
            'origin_x':   ox,
            'origin_y':   oy,
            'width':      width,
            'height':     height,
        }

        self.get_logger().debug(
            f"Stored occupancy grid: {width}×{height} cells @ {res:.3f} m/cell "
            f"origin=({ox:.2f}, {oy:.2f})")

    def handle_render_request(self, request, response):
        """
        Trigger service handler for /braille_heightmap/render_ego_crop.
        Snapshots the current pose, renders the forward-facing ego-centric
        crop, and publishes it to /braille/heightmap.
        """
        if self.full_occ_raw is None or self.full_occ_meta is None:
            response.success = False
            response.message = "No occupancy data stored yet."
            self.get_logger().warn("render_ego_crop: " + response.message)
            return response

        crop = self._render_ego_crop()

        if crop is None:
            # No pose yet — try the last valid snapshot as a fallback
            if self.latest_crop is not None:
                self.get_logger().warn(
                    "render_ego_crop: no pose available, publishing last valid crop.")
                self.publish_images(self.latest_crop, source='occupancy')
                response.success = True
                response.message = "Published last valid crop (no current pose)."
            else:
                response.success = False
                response.message = "No pose available and no fallback crop exists."
                self.get_logger().warn("render_ego_crop: " + response.message)
            return response

        self.latest_crop = crop
        self.publish_images(crop, source='occupancy')
        response.success = True
        response.message = "Ego-centric crop rendered and published."
        self.get_logger().info("render_ego_crop: " + response.message)
        return response

    def _render_ego_crop(self):
        """
        Extract a forward-facing, ego-centric window from the stored full
        occupancy grid.

        Window geometry
        ---------------
          - Camera sits at BOTTOM-CENTER of the display
          - Window spans ±(size/2) metres laterally (left/right of camera)
          - Window spans 0 → size metres forward (nothing behind camera)
          - Display is oriented so camera-forward always points to the top
            regardless of actual world heading  →  180° forward FOV only

        For each output pixel, the camera-local offset is rotated into world
        coordinates using the camera yaw, then the world occupancy map is
        sampled directly — no image rotation step needed.

        Returns a (DOT_ROWS, DOT_COLS) uint8 array, or None if pose is absent.
        """
        if self.latest_pose is None:
            return None

        pose   = self.latest_pose
        meta   = self.full_occ_meta
        res    = meta['resolution']
        ox, oy = meta['origin_x'], meta['origin_y']
        raw    = self.full_occ_raw

        cam_x = pose.position.x
        cam_y = pose.position.y

        q   = pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        half_w = self.size / 2.0  # lateral half-width (±left/right of camera)
        depth  = self.size        # forward depth       (0 → depth ahead)

        # Patch pixel dimensions
        patch_w = max(int(math.ceil(self.size / res)), 1)  # lateral axis
        patch_h = max(int(math.ceil(depth  / res)), 1)     # forward axis

        patch = np.full((patch_h, patch_w), 128, dtype=np.uint8)

        # Camera-local → world rotation components
        #   camera forward in world = ( cos_yaw,  sin_yaw )
        #   camera right   in world = ( sin_yaw, -cos_yaw )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for pr in range(patch_h):
            for pc in range(patch_w):
                # Camera-local offsets in metres:
                #   fwd_offset: pr=0 → full depth ahead; pr=patch_h-1 → ~0 (at camera)
                #   lat_offset: pc=0 → hard left (-half_w); pc=patch_w-1 → hard right (+half_w)
                fwd_offset =  depth  - (pr + 0.5) * res
                lat_offset = -half_w + (pc + 0.5) * res

                # Rotate offset into world frame
                wx = cam_x + fwd_offset * cos_yaw + lat_offset * sin_yaw
                wy = cam_y + fwd_offset * sin_yaw - lat_offset * cos_yaw

                # World coordinate → map pixel index
                mx = int((wx - ox) / res)
                my = int((wy - oy) / res)

                if 0 <= mx < meta['width'] and 0 <= my < meta['height']:
                    patch[pr, pc] = raw[my, mx]
                # else: leave as 128 (outside known map area)

        braille_map = self.preprocess_and_pad(patch, target_shape=(DOT_ROWS, DOT_COLS))
        return braille_map

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def publish_images(self, image, source=''):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing heightmap image: {e}")

        try:
            dbg_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if source == 'mesh':
                dbg_img = self.draw_pose_overlay(dbg_img, image.shape)
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg_img, encoding='bgr8')
            dbg_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(dbg_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

    def draw_pose_overlay(self, debug_image, grid_shape):
        if self.latest_pose:
            h, w = grid_shape
            q    = self.latest_pose.orientation
            x, y = self.latest_pose.position.x, self.latest_pose.position.y
            yaw  = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

            def world_to_px(wx, wy):
                half_size = self.size / 2.0
                return (
                    int((wx - (x - half_size)) / self.size * (w - 1)),
                    int((wy - (y - half_size)) / self.size * (h - 1))
                )

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

    # ------------------------------------------------------------------
    # Geometry / preprocessing utilities
    # ------------------------------------------------------------------

    def preprocess_and_pad(self, input_img, target_shape):
        h, w = input_img.shape
        target_h, target_w = target_shape
        input_aspect  = w / h
        target_aspect = target_w / target_h

        if input_aspect > target_aspect:
            new_w = target_w
            new_h = int(target_w / input_aspect)
        else:
            new_h = target_h
            new_w = int(target_h * input_aspect)

        resized = cv2.resize(input_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        padded  = np.full(target_shape, 128, dtype=np.uint8)
        y_offset = (target_h - new_h) // 2
        x_offset = (target_w - new_w) // 2
        padded[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized

        cv2.rectangle(
            padded,
            (x_offset, y_offset),
            (x_offset + new_w - 1, y_offset + new_h - 1),
            255, 1
        )
        return padded

    @staticmethod
    def generate_heightmap(vertices, grid_size=(60, 60), method='max',
                           center=None, size=5.0, yaw=0.0):
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


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

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