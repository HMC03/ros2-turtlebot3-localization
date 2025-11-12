import os
import yaml
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory

class MapLoader:
    def __init__(self):
        pkg_dir = get_package_share_directory('tb3_localization')
        yaml_path = os.path.join(pkg_dir, 'maps', 'map.yaml')
        df_path = os.path.join(pkg_dir, 'maps', 'distance_field.png')
        pgm_path = os.path.join(pkg_dir, 'maps', 'map.pgm')

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"map.yaml not found at {yaml_path}")
        if not os.path.exists(df_path):
            raise FileNotFoundError(f"distance_field.png not found at {df_path}")
        if not os.path.exists(pgm_path):
            raise FileNotFoundError(f"map.pgm not found at {pgm_path}")

        # Load YAML
        with open(yaml_path) as f:
            info = yaml.safe_load(f)

        self.resolution = info['resolution']
        self.origin = np.array(info['origin'][:2], dtype=np.float64)
        self.width = info.get('width', None)
        self.height = info.get('height', None)

        # Load distance field (your blurred masterpiece)
        df_img = cv2.imread(df_path, cv2.IMREAD_GRAYSCALE)
        if df_img is None:
            raise RuntimeError("Failed to load distance_field.png")
        self.dist_field = df_img.astype(np.float32)
        # Convert 0-255 → meters (you used 0-2.0m scale)
        self.dist_field = self.dist_field / 255.0 * 2.0

        # Load occupancy for validity
        occ_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if occ_img is None:
            raise RuntimeError("Failed to load map.pgm")
        # Threshold: dark = occupied
        self.occupancy = (occ_img < 100).astype(np.uint8)  # 1 = wall

        self.height, self.width = self.occupancy.shape

        # Validate sizes match
        if self.dist_field.shape != self.occupancy.shape:
            raise RuntimeError("distance_field.png and map.pgm have different dimensions!")

        # Precompute free cells for initialization
        self.free_y, self.free_x = np.where(self.occupancy == 0)
        if len(self.free_y) == 0:
            raise RuntimeError("No free space found in map!")

        print(f"Map loaded: {self.width}x{self.height} @ {self.resolution:.3f}m/cell")
        print(f"Origin: ({self.origin[0]:.2f}, {self.origin[1]:.2f})")
        print(f"Free cells: {len(self.free_y)}")

    def world_to_pixel(self, x, y):
        px = ((x - self.origin[0]) / self.resolution)
        py = ((y - self.origin[1]) / self.resolution)
        return int(px + 0.5), int(py + 0.5)

    def pixel_to_world_center(self, px, py):
        x = px * self.resolution + self.origin[0]
        y = py * self.resolution + self.origin[1]
        return x, y

    def is_valid_pose(self, x, y, margin_cells=2):
        px, py = self.world_to_pixel(x, y)
        if not (margin_cells <= px < self.width - margin_cells and
                margin_cells <= py < self.height - margin_cells):
            return False
        return self.occupancy[py, px] == 0

    def get_distance(self, x, y):
        px, py = self.world_to_pixel(x, y)
        if not (0 <= px < self.width and 0 <= py < self.height):
            return 2.0  # max distance
        return self.dist_field[py, px]

    def sample_free_pose(self):
        idx = np.random.randint(0, len(self.free_y))
        py, px = self.free_y[idx], self.free_x[idx]
        x, y = self.pixel_to_world_center(px, py)
        theta = np.random.uniform(-np.pi, np.pi)
        return x, y, theta