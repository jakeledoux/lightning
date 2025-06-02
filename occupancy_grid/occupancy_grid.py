import collections
import heapq
import os

import cv2
import numpy as np
import open3d as o3d

from record3d import Record3DStream
from threading import Event

# --- Global Configuration ---
WRITE_STEER = True
VISUALIZE = bool(os.environ.get("VISUALIZE"))  # Flag to enable/disable Open3D and OpenCV visualizations
RESET_VIEW_KEY = ord('R') # Key to reset the Open3D visualizer view


class PointCloudStreamer:
    """
    Handles streaming point cloud data from a Record3D sensor,
    processing it for floor alignment, generating an occupancy grid,
    performing pathfinding, and calculating steering commands.
    """
    def __init__(self, steer_file):
        self.steer_file = steer_file
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__LIDAR = 1 # Example device type constant

        # --- Alignment Parameters ---
        self.current_alignment_transform = np.eye(4)  # Stores the current floor alignment transform
        self.frames_since_last_align = 0
        self.align_interval = 3  # Recalculate alignment transform every N frames

        # --- Occupancy Grid Parameters ---
        self.grid_resolution = 0.02  # Meters per cell
        self.fixed_grid_display_width_m = 3.0  # Width of the fixed grid area in meters
        self.fixed_grid_display_depth_m = 3.0  # Depth of the fixed grid area in meters
        
        # Derived grid properties
        self.grid_num_cells_x = int(np.ceil(self.fixed_grid_display_width_m / self.grid_resolution))
        self.grid_num_cells_y = int(np.ceil(self.fixed_grid_display_depth_m / self.grid_resolution))
        self.grid_world_origin_x = -self.fixed_grid_display_width_m / 2.0 # World X for grid origin
        self.grid_world_origin_y = 0.0  # World Y for grid origin (camera's Y-plane is start of grid depth)

        # Obstacle definition for occupancy grid (assume-free-then-mark-obstacles model)
        self.obstacle_min_height_above_floor = 0.10 # Min height to be considered an obstacle
        self.obstacle_max_height_above_floor = 1.5  # Max height (ignore points above this)

        # Morphological processing for occupancy grid
        self.enable_path_clearance = True # If true, erodes free space to create margin around obstacles
        self.path_clearance_margin_cells = 5 # Thickness of margin (in cells)
        
        self.enable_morphological_processing = False # For general MORPH_CLOSE on floor (currently unused based on create_occupancy_grid logic)
        self.morph_kernel_size = 5
        self.morph_close_iterations = 1
        
        # --- Pathfinding Parameters ---
        self.current_path = None # Stores the last calculated path
        self.enable_path_smoothing = True
        self.pathfinding_interval = 5 # Recalculate path every N frames

        # --- Steering Parameters ---
        self.steering_lookahead_distance_cells = 5
        self.max_steering_angle_rad = np.pi / 7.5
        self.previous_smoothed_steering_command = 0.0
        self.steering_smoothing_alpha = 0.25 # For Exponential Moving Average (EMA)

        # --- Display Parameters ---
        self.occupancy_grid_window_name = "Occupancy Grid (OpenCV)"
        self.grid_cell_display_pixels = 2

    def write_steering(self, steer_value: float):
        self.steer_file.truncate(0)
        self.steer_file.seek(0)
        self.steer_file.write(f"{steer_value:0.2f}")
        self.steer_file.flush()

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        """Converts Record3D intrinsic coefficients to a 3x3 matrix."""
        return np.array(
            [[coeffs.fx, 0, coeffs.tx], 
             [0, coeffs.fy, coeffs.ty], 
             [0, 0, 1]]
        )

    def get_points(self):
        """
        Retrieves depth and RGB frames from the sensor, creates a point cloud,
        and applies an initial transformation for visualization.
        Assumes vertical (portrait) phone orientation.
        """
        depth_frame = self.session.get_depth_frame()
        rgb_frame = self.session.get_rgb_frame()

        if depth_frame is None or rgb_frame is None:
            return np.array([]), np.array([])

        intrinsics_coeffs = self.session.get_intrinsic_mat()
        intrinsic_mat = self.get_intrinsic_mat_from_coeffs(intrinsics_coeffs)

        target_height = rgb_frame.shape[0]
        target_width = rgb_frame.shape[1]

        if depth_frame.shape[0] != target_height or depth_frame.shape[1] != target_width:
            depth_frame_resized = cv2.resize(depth_frame, (target_width, target_height),
                                             interpolation=cv2.INTER_NEAREST)
        else:
            depth_frame_resized = depth_frame

        depth_o3d = o3d.geometry.Image(depth_frame_resized.astype(np.float32))
        rgb_o3d = o3d.geometry.Image(rgb_frame.astype(np.uint8))

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d, depth_scale=1.0, depth_trunc=3.0, convert_rgb_to_intensity=False
        )

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=target_width, height=target_height,
            fx=intrinsic_mat[0, 0], fy=intrinsic_mat[1, 1],
            cx=intrinsic_mat[0, 2], cy=intrinsic_mat[1, 2]
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # Standard view transform

        return np.asarray(pcd.points), np.asarray(pcd.colors)

    def _compute_floor_alignment_transform(self, pts_input,
                                           distance_threshold=0.02, ransac_n=3,
                                           num_iterations=100, voxel_size_for_plane_detection=0.05):
        """
        Computes the 4x4 transformation matrix to align the dominant plane (floor)
        to be parallel to the XY plane with its normal along Z.
        """
        if pts_input.size == 0:
            return None

        pcd_full = o3d.geometry.PointCloud()
        pcd_full.points = o3d.utility.Vector3dVector(pts_input)

        # Downsample for faster plane detection
        if voxel_size_for_plane_detection > 0:
            pcd_for_detection = pcd_full.voxel_down_sample(voxel_size=voxel_size_for_plane_detection)
            if len(pcd_for_detection.points) < ransac_n: return None
        else:
            pcd_for_detection = pcd_full
            if len(pcd_for_detection.points) < ransac_n: return None
        
        try:
            plane_model, inlier_indices = pcd_for_detection.segment_plane(
                distance_threshold=distance_threshold,
                ransac_n=ransac_n,
                num_iterations=num_iterations)
        except RuntimeError: 
            return None # RANSAC failed
        
        if plane_model is None or not inlier_indices: 
            return None 

        a, b, c, d_plane_eq = plane_model
        n_floor = np.array([a, b, c])
        n_floor_norm = np.linalg.norm(n_floor)

        if n_floor_norm < 1e-9: return None # Degenerate plane normal
        
        n_floor_unit = n_floor / n_floor_norm
        n_target = np.array([0.0, 0.0, 1.0]) # Target normal (Z-up)

        # Calculate rotation to align floor normal with target normal
        dot_product = np.clip(np.dot(n_floor_unit, n_target), -1.0, 1.0)
        R = np.eye(3) # Default to identity
        if dot_product > 0.99999: # Already aligned
            pass
        elif dot_product < -0.99999: # Exactly opposite
            R = o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([1.0,0.0,0.0]) * np.pi) # 180 deg rot around X
        else:
            axis = np.cross(n_floor_unit, n_target)
            axis_norm = np.linalg.norm(axis)
            if axis_norm > 1e-9: # Ensure non-zero axis for rotation
                R = o3d.geometry.get_rotation_matrix_from_axis_angle((axis / axis_norm) * np.arccos(dot_product))
        
        T_rotate = np.eye(4); T_rotate[:3,:3] = R

        # Calculate translation to move the detected plane to Z=0
        # Uses centroid of inliers from the original full point cloud for accuracy
        distances_to_plane = pts_input @ n_floor + d_plane_eq # n_floor from RANSAC, d_plane_eq from RANSAC
        original_inlier_indices = np.where(np.abs(distances_to_plane) < distance_threshold)[0]

        translation_z = 0.0
        if original_inlier_indices.size > 0:
            original_inlier_points = pts_input[original_inlier_indices]
            rotated_original_inlier_points = (R @ original_inlier_points.T).T
            centroid_rotated_original_inliers = np.mean(rotated_original_inlier_points, axis=0)
            translation_z = -centroid_rotated_original_inliers[2]
        
        T_translate = np.eye(4); T_translate[2,3] = translation_z
        
        return T_translate @ T_rotate # Combined transform: Rotate then Translate

    def create_occupancy_grid(self, aligned_pts):
        """
        Creates a 2D occupancy grid (1=traversable, 0=obstacle) by assuming the area
        is free and then marking detected obstacles based on height above the aligned floor.
        Applies path clearance by eroding free space (dilating obstacles).
        """
        # Initialize grid as all traversable
        current_occupancy_grid = np.ones((self.grid_num_cells_y, self.grid_num_cells_x), dtype=np.uint8)

        if aligned_pts.size == 0:
            return current_occupancy_grid

        # Identify obstacle points based on height thresholds
        obstacle_points_mask = (aligned_pts[:, 2] > self.obstacle_min_height_above_floor) & \
                               (aligned_pts[:, 2] < self.obstacle_max_height_above_floor)
        obstacle_points = aligned_pts[obstacle_points_mask]

        if obstacle_points.shape[0] > 0:
            # Filter obstacles within fixed grid boundaries
            grid_world_max_x = self.grid_world_origin_x + self.fixed_grid_display_width_m
            grid_world_max_y = self.grid_world_origin_y + self.fixed_grid_display_depth_m

            x_valid_mask = (obstacle_points[:, 0] >= self.grid_world_origin_x) & \
                           (obstacle_points[:, 0] < grid_world_max_x)
            y_valid_mask = (obstacle_points[:, 1] >= self.grid_world_origin_y) & \
                           (obstacle_points[:, 1] < grid_world_max_y)
            obstacle_points_in_grid_area = obstacle_points[x_valid_mask & y_valid_mask]

            if obstacle_points_in_grid_area.shape[0] > 0:
                # Mark obstacle cells as 0
                obs_cell_x_indices = np.floor(
                    (obstacle_points_in_grid_area[:, 0] - self.grid_world_origin_x) / self.grid_resolution
                ).astype(int)
                obs_cell_y_indices = np.floor(
                    (obstacle_points_in_grid_area[:, 1] - self.grid_world_origin_y) / self.grid_resolution
                ).astype(int)

                obs_cell_x_indices = np.clip(obs_cell_x_indices, 0, self.grid_num_cells_x - 1)
                obs_cell_y_indices = np.clip(obs_cell_y_indices, 0, self.grid_num_cells_y - 1)
                current_occupancy_grid[obs_cell_y_indices, obs_cell_x_indices] = 0
        
        # Note: self.enable_morphological_processing for MORPH_CLOSE on floor is not used here
        # as this function now assumes free then marks obstacles.
        # If MORPH_CLOSE was intended for the obstacle mask, that would be a different logic.

        # Path Clearance: Erode free space (effectively dilate obstacles)
        if self.enable_path_clearance and self.path_clearance_margin_cells > 0:
            # Ensure there are actual obstacles to dilate before proceeding
            if np.any(current_occupancy_grid == 0): 
                kernel_dim = 2 * self.path_clearance_margin_cells + 1
                if kernel_dim >= 3: # Sensible kernel size
                    kernel = np.ones((kernel_dim, kernel_dim), np.uint8)
                    current_occupancy_grid = cv2.erode(current_occupancy_grid, kernel, iterations=1)
        
        return current_occupancy_grid

    def _find_dynamic_start_cell(self, traversable_grid):
        """
        Finds a start cell on the traversable floor, prioritizing the center of the
        row closest to the camera (row 0 in grid data, bottom of display after flip).
        """
        if traversable_grid is None or traversable_grid.size == 0: return None
        rows, cols = traversable_grid.shape
        
        center_col = cols // 2
        # Search a limited number of rows from the "bottom" (row 0 of grid data)
        max_rows_to_search_start = max(1, int(rows * 0.15)) 

        for r in range(max_rows_to_search_start):
            if r >= rows: break 
            if traversable_grid[r, center_col] == 1: return (r, center_col)
            
            search_half_width_cells = max(1, cols // 10) 
            for dc_offset in range(1, search_half_width_cells + 1):
                if center_col + dc_offset < cols and traversable_grid[r, center_col + dc_offset] == 1:
                    return (r, center_col + dc_offset)
                if center_col - dc_offset >= 0 and traversable_grid[r, center_col - dc_offset] == 1:
                    return (r, center_col - dc_offset)
        return None

    def generate_wide_forward_path(self, traversable_grid, dist_transform, start_cell, max_path_cells=100):
        """
        Generates a path from start_cell, moving forward while preferring wider areas.
        """
        if start_cell is None or traversable_grid[start_cell[0], start_cell[1]] == 0:
            return None

        rows, cols = traversable_grid.shape
        path = [start_cell]
        current_cell = start_cell
        grid_center_col = cols // 2

        for _ in range(max_path_cells):
            best_next_cell = None
            max_score = -float('inf')

            # Candidate moves: (delta_row, delta_col)
            # Positive delta_row moves "forward" in grid data (towards top of display after flip)
            moves = [
                (1, 0),   # Straight Forward
                (1, -1),  # Forward-Left Diagonal
                (1, 1),   # Forward-Right Diagonal
                (0, -1),  # Left
                (0, 1),   # Right
            ]

            possible_moves_evaluated = 0
            for dr_iter, dc_iter in moves:
                nr, nc = current_cell[0] + dr_iter, current_cell[1] + dc_iter

                if not (0 <= nr < rows and 0 <= nc < cols and traversable_grid[nr, nc] == 1):
                    continue
                possible_moves_evaluated += 1
                
                score = 0.0
                if dr_iter > 0: score += 80.0 # Reduced forward bias
                elif dr_iter == 0: score += 20.0 # Sideways
                if dr_iter > 0 and dc_iter == 0: score += 40.0 # Straight forward bonus

                dt_val = dist_transform[nr, nc]
                score += dt_val * 50.0 # Weight for distance transform (width)

                min_dt_for_comfort = 2.5 
                if dt_val < min_dt_for_comfort:
                    score -= (min_dt_for_comfort - dt_val) * 150.0 # Penalty for being too close

                score -= abs(nc - grid_center_col) * 0.1 # Center seeking (small penalty)
                if len(path) > 1 and (nr, nc) == path[-2]: score -= 500.0 # Avoid immediate U-turn

                if score > max_score:
                    max_score = score
                    best_next_cell = (nr, nc)
            
            if best_next_cell is None: break
            if possible_moves_evaluated > 0 and max_score < -100.0: break # Stuck with bad options
            if len(path) > 2 and best_next_cell == path[-2] and best_next_cell[0] <= current_cell[0]: break # Wiggling

            path.append(best_next_cell)
            current_cell = best_next_cell
            
            # Stop condition considering clearance margin
            stop_margin = 1
            if self.enable_path_clearance and self.path_clearance_margin_cells > 0:
                 stop_margin = 1 + self.path_clearance_margin_cells
            if current_cell[0] >= rows - stop_margin: break
        
        return path if len(path) >= 2 else None

    def _has_line_of_sight(self, occupancy_grid, p1_cell, p2_cell):
        """Checks line of sight on the occupancy grid (1 is traversable)."""
        r1, c1 = p1_cell; r2, c2 = p2_cell
        dr_total = abs(r2 - r1); dc_total = abs(c2 - c1)
        sr = 1 if r1 < r2 else -1; sc = 1 if c1 < c2 else -1
        
        # Use a set for visited cells to handle diagonal movements more accurately
        # in a discrete grid for line of sight. Bresenham is tricky for this.
        # Simpler: sample points along the line.
        num_samples = max(dr_total, dc_total) + 1 # Number of points to check along the line
        if num_samples <=1 : return True # Points are same or adjacent

        for i in range(num_samples + 1): # Check p1 and p2 as well for safety
            t = i / float(num_samples)
            r = int(round(r1 + t * (r2 - r1)))
            c = int(round(c1 + t * (c2 - c1)))

            if not (0 <= r < occupancy_grid.shape[0] and 0 <= c < occupancy_grid.shape[1]):
                return False # Line goes out of bounds
            if occupancy_grid[r, c] == 0: # Hits an obstacle
                return False
        return True

    def smooth_path_shortcut(self, occupancy_grid, path):
        """Smooths a path using the shortcut method."""
        if not path or len(path) < 3: return path

        smoothed_path = [path[0]]
        current_original_idx = 0
        while current_original_idx < len(path) - 1:
            last_smooth_point = smoothed_path[-1]
            best_shortcut_idx = current_original_idx + 1 # Default: next point in original path

            # Look from the end of the original path backwards for a shortcut
            for test_idx_in_original in range(len(path) - 1, current_original_idx + 1, -1):
                if self._has_line_of_sight(occupancy_grid, last_smooth_point, path[test_idx_in_original]):
                    best_shortcut_idx = test_idx_in_original
                    break # Found the furthest visible point
            
            smoothed_path.append(path[best_shortcut_idx])
            current_original_idx = best_shortcut_idx
        return smoothed_path

    def get_steering_command(self, path):
        """Calculates steering command (-1 to +1) from the path."""
        if path is None or len(path) < 2: return 0.0

        current_pos_grid = path[0]
        lookahead_idx = min(len(path) - 1, self.steering_lookahead_distance_cells)
        if lookahead_idx == 0: # Path is effectively just current_pos_grid
            return 0.0 
        target_pos_grid = path[lookahead_idx]

        dr = float(target_pos_grid[0] - current_pos_grid[0]) # Forward
        dc = float(target_pos_grid[1] - current_pos_grid[1]) # Sideways

        if dr <= 0: # Target not forward or purely sideways
            return (1.0 if dc > 0 else -1.0) if dr == 0 and dc != 0 else 0.0
        
        angle_to_target_rad = np.arctan2(dc, dr)
        steering_command = np.clip(angle_to_target_rad / self.max_steering_angle_rad, -1.0, 1.0)
        return steering_command

    def display_occupancy_grid_cv2(self, occupancy_grid, path, steering_cmd=None): # Added steering_cmd argument
        """Displays the occupancy grid, path, and steering command using OpenCV."""
        if occupancy_grid is None:
            # Create a default sized black image if no grid data
            img_h = self.grid_num_cells_y * self.grid_cell_display_pixels
            img_w = self.grid_num_cells_x * self.grid_cell_display_pixels
            img_h = max(img_h, 50) # Ensure minimum size
            img_w = max(img_w, 50)
            final_display_image = np.zeros((img_h, img_w, 3), dtype=np.uint8)
        else:
            mono_image = (occupancy_grid * 255).astype(np.uint8)
            scaled_h = self.grid_num_cells_y * self.grid_cell_display_pixels
            scaled_w = self.grid_num_cells_x * self.grid_cell_display_pixels

            if scaled_h > 0 and scaled_w > 0:
                display_image_scaled = cv2.resize(mono_image, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST)
            else: # Fallback for tiny/empty grid data
                display_image_scaled = np.zeros((50,50), dtype=np.uint8) if mono_image.size == 0 else mono_image
            
            display_image_bgr = cv2.cvtColor(display_image_scaled, cv2.COLOR_GRAY2BGR)

            if path and len(path) > 1:
                path_color = (0, 255, 0) # Green
                thickness = max(1, self.grid_cell_display_pixels // 4)
                for i in range(len(path) - 1):
                    p1 = (path[i][1] * self.grid_cell_display_pixels + self.grid_cell_display_pixels // 2,
                          path[i][0] * self.grid_cell_display_pixels + self.grid_cell_display_pixels // 2)
                    p2 = (path[i+1][1] * self.grid_cell_display_pixels + self.grid_cell_display_pixels // 2,
                          path[i+1][0] * self.grid_cell_display_pixels + self.grid_cell_display_pixels // 2)
                    cv2.line(display_image_bgr, p1, p2, path_color, thickness)
            
            final_display_image = cv2.flip(display_image_bgr, 0) # Flip after drawing path

            # Draw Camera Marker
            cam_px = final_display_image.shape[1] // 2
            cam_py = final_display_image.shape[0] - (self.grid_cell_display_pixels // 2)
            cv2.circle(final_display_image, (cam_px, cam_py), 
                       max(2, self.grid_cell_display_pixels // 3), (0,0,255), -1)

            # Draw Goal Marker if path exists
            if path and path[-1]:
                goal = path[-1]
                goal_px = goal[1] * self.grid_cell_display_pixels + self.grid_cell_display_pixels // 2
                goal_py_unflipped = goal[0] * self.grid_cell_display_pixels + self.grid_cell_display_pixels // 2
                goal_py_flipped = final_display_image.shape[0] - 1 - goal_py_unflipped
                cv2.circle(final_display_image, (goal_px, goal_py_flipped),
                           max(2, self.grid_cell_display_pixels // 3), (255,0,0), -1)
        
        if steering_cmd is not None:
             cv2.putText(final_display_image, f"Steer: {steering_cmd:.2f}",
                        (10, final_display_image.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(self.occupancy_grid_window_name, final_display_image)

    def _recenter_view_callback(self, vis):
        """Callback to reset the Open3D visualizer view."""
        print("Key 'R' pressed - Recenter_viewing view.")
        vis.reset_view_point(True)
        return False

    def on_new_frame(self):
        """Callback for when Record3D has a new frame."""
        self.event.set()

    def on_stream_stopped(self):
        """Callback for when the Record3D stream stops."""
        print("Stream stopped")

    def connect_to_device(self, dev_idx):
        """Connects to the Record3D device at the given index."""
        print("Searching for devices")
        devs = Record3DStream.get_connected_devices()
        print(f"{len(devs)} device(s) found")
        for dev in devs:
            print(f"\tID: {dev.product_id}\n\tUDID: {dev.udid}\n")

        if len(devs) <= dev_idx:
            raise RuntimeError(f"Cannot connect to device #{dev_idx}, try different index.")

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)
        print(f"Connected to device: {dev.udid}")

    def start_processing_stream(self):
        """Main loop for processing and visualizing the point cloud stream."""
        pcd_o3d = o3d.geometry.PointCloud() # Renamed for clarity (Open3D PointCloud)
        pcd_o3d.points = o3d.utility.Vector3dVector([[0,0,0]]) # Dummy point
        pcd_o3d.colors = o3d.utility.Vector3dVector([[0,0,0]])

        if VISUALIZE:
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window(window_name="Live Point Cloud", width=800, height=600)
            vis.add_geometry(pcd_o3d)
            vis.register_key_callback(RESET_VIEW_KEY, self._recenter_view_callback)
            print("INFO: Press 'R' in the Open3D visualizer window to recenter the view.")
            print("      Press 'Esc' in the OpenCV window to quit.")
            
            # Setup initial Open3D view
            view_control = vis.get_view_control()
            view_control.set_zoom(0.7)
            view_control.set_front([0.5, -1, -0.3]) 
            view_control.set_lookat([0, 0, 0.5])    
            view_control.set_up([0, 0, 1]) # Z-axis is 'up' after floor alignment

        pathfinding_frame_counter = 0
        # Initialize to trigger alignment on first valid frame
        self.frames_since_last_align = self.align_interval 

        while True:
            self.event.wait() # Wait for new frame signal

            try:
                pts_raw, colors_raw = self.get_points()

                if pts_raw.size == 0:
                    if VISUALIZE:
                        # Display empty grid if no points, potentially with last path/steering
                        empty_grid = self.create_occupancy_grid(np.array([]))
                        self.display_occupancy_grid_cv2(empty_grid, self.current_path, self.previous_smoothed_steering_command)
                    self.event.clear()
                    continue

                # --- Floor Alignment ---
                self.frames_since_last_align += 1
                if self.frames_since_last_align >= self.align_interval:
                    new_transform = self._compute_floor_alignment_transform(pts_raw)
                    if new_transform is not None:
                        self.current_alignment_transform = new_transform
                    # else: keep using the old transform if alignment failed
                    self.frames_since_last_align = 0
                
                pcd_to_transform = o3d.geometry.PointCloud()
                pcd_to_transform.points = o3d.utility.Vector3dVector(pts_raw)
                pcd_to_transform.transform(self.current_alignment_transform)
                pts_to_display = np.asarray(pcd_to_transform.points)

                # --- Occupancy Grid ---
                traversable_grid = self.create_occupancy_grid(pts_to_display)
                
                # --- Pathfinding ---
                pathfinding_frame_counter += 1
                if traversable_grid is not None and traversable_grid.size > 0 and \
                   pathfinding_frame_counter >= self.pathfinding_interval:
                    pathfinding_frame_counter = 0
                    
                    dist_transform = cv2.distanceTransform(traversable_grid.astype(np.uint8), 
                                                           cv2.DIST_L2, maskSize=5)
                    dynamic_start_cell = self._find_dynamic_start_cell(traversable_grid)
                    
                    temp_path = None
                    if dynamic_start_cell:
                        temp_path = self.generate_wide_forward_path(
                            traversable_grid, dist_transform, dynamic_start_cell,
                            max_path_cells=self.grid_num_cells_y * 2 # Allow path to span grid depth
                        )
                        if temp_path and self.enable_path_smoothing:
                            temp_path = self.smooth_path_shortcut(traversable_grid, temp_path)
                    self.current_path = temp_path

                # --- Steering Command ---
                raw_steering_command = 0.0
                if self.current_path:
                    raw_steering_command = self.get_steering_command(self.current_path)
                
                smoothed_steering_command = \
                    (self.steering_smoothing_alpha * raw_steering_command) + \
                    ((1 - self.steering_smoothing_alpha) * self.previous_smoothed_steering_command)
                self.previous_smoothed_steering_command = smoothed_steering_command
                
                # Console output for steering
                # Consider printing less frequently if too verbose
                if pathfinding_frame_counter == 0: # Print when pathfinding attempts to run
                     print(f"Path found: {'Yes' if self.current_path else 'No'}, Steering: {smoothed_steering_command:.2f}")
                     if WRITE_STEER:
                         self.write_steering(smoothed_steering_command)


                # --- Display ---
                if VISUALIZE:
                    self.display_occupancy_grid_cv2(traversable_grid, self.current_path, smoothed_steering_command)
                    
                    pcd_o3d.points = o3d.utility.Vector3dVector(pts_to_display if pts_to_display.size >0 else [[0,0,0]])
                    pcd_o3d.colors = o3d.utility.Vector3dVector(colors_raw if colors_raw.size > 0 and pts_to_display.size > 0 else [[0,0,0]])
                    vis.update_geometry(pcd_o3d)

            except Exception as e:
                print(f"Error processing frame: {e}")
                import traceback
                traceback.print_exc()
            finally:
                self.event.clear()

            if VISUALIZE:
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC key
                    print("Escape key pressed. Exiting...")
                    break
                if not vis.poll_events(): # Open3D window closed
                    break
                vis.update_renderer()
            else: # Headless mode, maybe a delay or different exit condition
                # For now, if not visualizing, it will free-run. Add Ctrl+C handler for headless.
                pass 

        # --- Cleanup ---
        if VISUALIZE:
            vis.destroy_window()
            cv2.destroyAllWindows()
        print("Exiting PointCloudStreamer.")


if __name__ == "__main__":
    with open("/tmp/lightning-steer", "w") as f:
        streamer = PointCloudStreamer(f)
        try:
            streamer.connect_to_device(dev_idx=0)
            streamer.start_processing_stream()
        except RuntimeError as e:
            print(e)
        except KeyboardInterrupt:
            print("Stream stopped by user (Ctrl+C).")
        finally:
            if streamer.session is not None:
                # Make sure to stop the session if it was started
                # This might be handled by Record3D's __del__ or similar,
                # but explicit stop/disconnect is safer if available.
                # streamer.session.stop() # If such a method exists
                print("Ensuring Record3D session is cleaned up if possible.")
            print("Program terminated.")
