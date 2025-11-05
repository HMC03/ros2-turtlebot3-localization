#!/usr/bin/env python3
"""
motion_and_sensor_node.py

- Uses rclpy to subscribe to /odom and /scan
- Implements odometry-based motion model sampling
- Loads map (map.pgm + map.yaml), computes distance transform (distance_field.png)
- Computes likelihood of each incoming scan for the current (sampled) pose

Run with:
ros2 run <your_package> motion_and_sensor_node
(or run directly with python if your PYTHONPATH contains rclpy)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import numpy as np
import math, time, os, yaml
import ament_index_python.packages as pkg_index

# imaging libs
try:
    import cv2
except Exception:
    cv2 = None
try:
    from scipy.ndimage import distance_transform_edt
except Exception:
    distance_transform_edt = None
from PIL import Image

# ---------------------------
# Utility: load map and compute distance field
# ---------------------------
class MapUtils:
    def __init__(self, map_yaml_path):
        # map_yaml_path: path to the .yaml file produced by map_saver
        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)
        img_path = os.path.join(os.path.dirname(map_yaml_path), info['image'])
        self.resolution = float(info['resolution'])
        self.origin = tuple(info['origin'])  # [origin_x, origin_y, yaw]
        self.negate = info.get('negate', 0)
        self.occupied_thresh = info.get('occupied_thresh', 0.65)
        self.free_thresh = info.get('free_thresh', 0.196)

        # load image as grayscale
        img = None
        if cv2 is not None:
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise RuntimeError(f"Failed to load map image {img_path} via cv2")
            # cv2 loads with row 0 at top
            self.map_img = img
        else:
            im = Image.open(img_path).convert('L')
            self.map_img = np.array(im)

        # threshold to binary obstacle map: obstacles = 1, free = 0
        # map server uses: 0 = free, 205 = unknown, 254 = occupied typically,
        # but we just threshold: occupied if pixel <= occupied_thresh*255 after considering negate.
        # adjust for negate
        if self.negate:
            processed = 255 - self.map_img
        else:
            processed = self.map_img.copy()

        # occupied where pixel value > (occupied_thresh*255) ??? map_server usually sets high values for occupied.
        # We will treat darker values as free and bright as occupied depending on how map was generated.
        # A reliable approach: use nav2 style: occupied if pixel <= occupied_thresh*255 when negate=0
        # but empirically map images from nav2 often have 0 free, 205 unknown, 254 occupied.
        # We'll choose a heuristic: occupied if pixel >= 250
        self.occ_mask = (processed <= 50).astype(np.uint8)  # 1 for obstacles

        # compute distance transform in meters
        self.distance_field = self._compute_distance_field(self.occ_mask)  # same shape, units meters

        self.height, self.width = self.map_img.shape

    def _compute_distance_field(self, occ_mask):
        # occ_mask: 1 where obstacle, 0 elsewhere
        if distance_transform_edt is not None:
            # distance from free pixel to nearest obstacle (in pixels)
            inv = 1 - occ_mask  # free=1, occ=0 for edt
            dist_px = distance_transform_edt(inv == 1)  # distance to nearest zero (obstacle)
            # convert pixels -> meters
            dist_m = dist_px * self.resolution
            return dist_m
        elif cv2 is not None:
            # use cv2.distanceTransform; need 8-bit image where 0 is background -> obstacle should be 0
            # We set obstacle pixels to 0, free pixels to 255
            img = (1 - occ_mask) * 255  # free=255, obs=0
            img = img.astype('uint8')
            dist = cv2.distanceTransform(img, distanceType=cv2.DIST_L2, maskSize=5)
            return dist * self.resolution
        else:
            # slow fallback: brute-force distance (O(N^2)) — okay for small maps
            H, W = occ_mask.shape
            coords = np.column_stack(np.where(occ_mask == 1))
            if coords.size == 0:
                return np.full((H, W), np.inf)
            yy = np.arange(H)[:, None]
            xx = np.arange(W)[None, :]
            grid = np.stack(np.meshgrid(xx, yy), axis=-1)  # (H,W,2)
            df = np.zeros((H, W), dtype=np.float32)
            for r in range(H):
                for c in range(W):
                    # euclidean distance to nearest occupied
                    dmin = np.min((coords[:,0] - r)**2 + (coords[:,1] - c)**2)
                    df[r, c] = math.sqrt(dmin) * self.resolution
            return df

    def save_distance_field_png(self, out_path):
        """
        Save distance field to grayscale PNG for visualization.
        Uses a fixed physical scale (0–2.0 m) for consistent shading.
        Obstacles = white, open space = dark gray.
        """
        df = self.distance_field.copy()

        # Choose a fixed visualization range in meters
        # This prevents normalization from changing between maps
        max_m = 0.5  # how far from walls the gradient extends visibly

        # Clip and normalize linearly
        vis = np.clip(df, 0, max_m) / max_m

        # Invert so walls are white and free space is darker
        vis = 1.0 - vis

        # Convert to 0–255 grayscale
        vis = (vis * 255).astype(np.uint8)

        # Force obstacles to pure white
        vis[self.occ_mask == 1] = 255

        Image.fromarray(vis).save(out_path)
        return out_path

    def world_to_map(self, x, y):
        """
        Convert world coords (x,y) to map pixel indices (row, col).
        Note: map image has row 0 at top. map origin (in YAML) corresponds to pixel (0,0) at lower-left.
        We therefore need to flip the y axis.
        """
        ox, oy, yaw = self.origin
        col = int((x - ox) / self.resolution + 0.5)
        row_from_bottom = int((y - oy) / self.resolution + 0.5)
        row = self.height - 1 - row_from_bottom
        return row, col

    def get_distance_at(self, x, y):
        r, c = self.world_to_map(x, y)
        if 0 <= r < self.height and 0 <= c < self.width:
            return float(self.distance_field[r, c])
        else:
            # outside map = large distance
            return float(np.max(self.distance_field))


# ---------------------------
# Motion model helper (odometry-based)
# ---------------------------
def angle_normalize(a):
    return math.atan2(math.sin(a), math.cos(a))

def sample_motion_model_odometry(prev_pose, new_odom_pose, alphas):
    """
    prev_pose, new_odom_pose: tuples (x, y, theta) from odometry
    alphas: list of 4 noise parameters [a1,a2,a3,a4]
    returns: sampled_pose (x,y,theta)
    Implementation follows Probabilistic Robotics odometry motion model.
    """
    x0, y0, th0 = prev_pose
    x1, y1, th1 = new_odom_pose

    # compute odometry-based motion
    dx = x1 - x0
    dy = y1 - y0
    trans = math.sqrt(dx * dx + dy * dy)
    rot1 = angle_normalize(math.atan2(dy, dx) - th0)
    rot2 = angle_normalize(th1 - th0 - rot1)

    a1, a2, a3, a4 = alphas

    # sample noise
    rot1_hat = rot1 - np.random.normal(0.0, math.sqrt(a1 * rot1 * rot1 + a2 * trans * trans))
    trans_hat = trans - np.random.normal(0.0, math.sqrt(a3 * trans * trans + a4 * (rot1 * rot1 + rot2 * rot2)))
    rot2_hat = rot2 - np.random.normal(0.0, math.sqrt(a1 * rot2 * rot2 + a2 * trans * trans))

    x_hat = x0 + trans_hat * math.cos(th0 + rot1_hat)
    y_hat = y0 + trans_hat * math.sin(th0 + rot1_hat)
    th_hat = angle_normalize(th0 + rot1_hat + rot2_hat)
    return (x_hat, y_hat, th_hat)

# ---------------------------
# The ROS2 node
# ---------------------------
class MotionSensorNode(Node):
    def __init__(self, map_yaml_path):
        super().__init__('motion_sensor_node')
        self.get_logger().info('motion_sensor_node starting')

        # odom bookkeeping
        self.last_odom = None  # (x,y,theta)
        self.last_odom_raw = None

        # motion model alphas (tune these)
        self.alphas = [0.001, 0.001, 0.01, 0.01]  # a1..a4

        # distance field
        self.map_utils = MapUtils(map_yaml_path)
        out_df = os.path.join(os.path.dirname(map_yaml_path), 'distance_field.png')
        self.get_logger().info(f'Writing distance field to {out_df}')
        self.map_utils.save_distance_field_png(out_df)

        # subsample beams to speed up likelihood computation
        self.beam_subsample = 5  # use every 5th beam; adjust as needed

        # scan likelihood publisher
        self.likelihood_pub = self.create_publisher(Float64, '/scan_likelihood', 1)

        # subs
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

    def odom_cb(self, msg: Odometry):
        # extract pose
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        # quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        odom_pose = (px, py, yaw)

        if self.last_odom is None:
            self.last_odom = odom_pose
            self.last_odom_raw = odom_pose
            return

        # sample new pose from motion model using odometry delta
        sampled = sample_motion_model_odometry(self.last_odom, odom_pose, self.alphas)
        self.get_logger().debug(f'sampled pose: {sampled}')
        self.last_odom = odom_pose
        # For now we just log; this is where you'd integrate into a particle filter or odometry predictor

    def scan_cb(self, msg: LaserScan):
        # compute likelihood of this scan for the latest odom pose
        if self.last_odom is None:
            self.get_logger().warn('No odom yet; skipping scan likelihood')
            return
        pose = self.last_odom  # use odom pose; you can use sampled pose if you prefer
        log_likelihood = 0.0
        sigma_hit = 0.2  # meters; measurement noise
        max_range = msg.range_max
        z_rand = 0.05  # small random measurement weight

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % self.beam_subsample != 0:
                angle += msg.angle_increment
                continue
            if math.isinf(r) or math.isnan(r) or r <= msg.range_min:
                angle += msg.angle_increment
                continue
            # endpoint in world frame given robot pose
            beam_angle = pose[2] + angle
            ex = pose[0] + r * math.cos(beam_angle)
            ey = pose[1] + r * math.sin(beam_angle)
            dist = self.map_utils.get_distance_at(ex, ey)  # meters
            p_hit = math.exp(- (dist * dist) / (2.0 * sigma_hit * sigma_hit))
            p = (1.0 - z_rand) * p_hit + z_rand * (1.0 / max_range)
            # avoid log(0)
            p = max(p, 1e-12)
            log_likelihood += math.log(p)
            angle += msg.angle_increment

        likelihood = math.exp(log_likelihood / max(1, len(msg.ranges)/self.beam_subsample))
        self.get_logger().info(f'scan likelihood (avg, beams subsampled) ~ {likelihood:.6e}')
        self.likelihood_pub.publish(Float64(data=float(likelihood)))


def main(args=None):

    rclpy.init(args=args)

    # Try to find the package share directory dynamically
    try:
        package_share = pkg_index.get_package_share_directory('tb3_localization')
    except Exception as e:
        raise RuntimeError("Could not locate the 'tb3_localization' package. "
                           "Ensure it is built and sourced before running.") from e

    # Construct the path to the map.yaml
    map_yaml = os.path.join(package_share, 'maps', 'map.yaml')

    if not os.path.exists(map_yaml):
        raise FileNotFoundError(f"Could not find map.yaml at {map_yaml}. "
                                "Make sure you have saved a map into tb3_localization/maps/")

    node = MotionSensorNode(map_yaml)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
