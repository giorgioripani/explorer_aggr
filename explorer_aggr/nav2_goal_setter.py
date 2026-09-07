# NumPy for vectorized grid processing and spatial array operations
import numpy as np
# Standard math module for geometric and trigonometric operations
import math

class GoalSetter:
    """
    Identifies, clusters, and scores exploration frontiers from 2D occupancy grids.
    """
    def __init__(self):
        # Map target pixel coordinates (px, py) to failure counts
        self.failed_goals = {}
        # Cache most recent target pixel to register failure if navigation aborts
        self.last_target_pixel = None

    def register_failure(self, x, y):
        """
        Increment failure penalty count for the active target pixel upon navigation failure.
        """
        # Penalize the pixel index of the last dispatched goal
        if self.last_target_pixel is not None:
            if self.last_target_pixel in self.failed_goals:
                self.failed_goals[self.last_target_pixel] += 1
            else:
                self.failed_goals[self.last_target_pixel] = 1
                
            print(f"[Matematico] Registrato fallimento per il PIXEL {self.last_target_pixel}. Totale: {self.failed_goals[self.last_target_pixel]}")




    def group_frontiers(self, frontier_indices, width, tolerance=2, min_size=25):
        """
        Cluster adjacent frontier cells using BFS connected component analysis and filter noise below min_size.
        """
        # Populate unvisited set with (x, y) grid coordinates converted from 1D flat indices
        unvisited = set()
        for idx in frontier_indices:
            px_x = int(idx % width)
            px_y = int(idx // width)
            unvisited.add((px_x, px_y))

        clusters = []

        # Iterate until all detected frontier points are clustered
        while unvisited:
            # Seed a new cluster from an arbitrary unvisited cell
            start_p = unvisited.pop()
            current_cluster = [start_p]
            queue = [start_p]
            
            # Breadth-first search queue traversal
            while queue:
                cx, cy = queue.pop(0)
                
                # Check Moore neighborhood defined by tolerance radius
                for dx in range(-tolerance, tolerance + 1):
                    for dy in range(-tolerance, tolerance + 1):
                        neighbor = (cx + dx, cy + dy)
                        
                        # Add adjacent connected frontier cells to current cluster
                        if neighbor in unvisited:
                            unvisited.remove(neighbor)
                            current_cluster.append(neighbor)
                            queue.append(neighbor)
                            
            # Filter clusters below minimum size threshold to reject sensor noise
            if len(current_cluster) >= min_size:
                clusters.append(current_cluster)

        return clusters






    def calculate_best_frontier(self, map_msg, robot_x_in_map, robot_y_in_map):
            # 1. Map metadata extraction for spatial transformations
            width = map_msg.info.width
            height = map_msg.info.height
            resolution = map_msg.info.resolution
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y

            # Guard clause: defer computation if grid dimensions are uninitialized
            if width == 0 or height == 0:
                return "WAIT", "WAIT"

            # 2. Vectorized 2D grid restructuring
            map_array = np.array(map_msg.data, dtype=np.int8)
            grid2d = map_array.reshape((height, width))
            
            # Binary masks for free (0) and unknown (-1) cells
            free2d = (grid2d == 0)
            unknown2d = (grid2d == -1)
            
            # 4-connectivity spatial shift to flag free cells bordering unknown space
            has_unknown_neighbor = np.zeros_like(free2d, dtype=bool)
            
            has_unknown_neighbor[:-1, :] |= unknown2d[1:, :]   # Top neighbor
            has_unknown_neighbor[1:, :] |= unknown2d[:-1, :]   # Bottom neighbor
            has_unknown_neighbor[:, :-1] |= unknown2d[:, 1:]   # Right neighbor
            has_unknown_neighbor[:, 1:] |= unknown2d[:, :-1]   # Left neighbor
            
            # Frontier cells: free space directly adjacent to unexplored space
            frontier_mask2d = free2d & has_unknown_neighbor
            
            # Suppress map boundaries to eliminate border artifacts
            frontier_mask2d[0, :] = False
            frontier_mask2d[-1, :] = False
            frontier_mask2d[:, 0] = False
            frontier_mask2d[:, -1] = False
            
            # Extract 2D grid coordinates of active frontier cells
            y_indices, x_indices = np.where(frontier_mask2d)
            
            # Convert 2D indices back to 1D flat representation: (Y * width) + X
            frontier_indices = list((y_indices * width) + x_indices)

            print(f"[Matematico] Trovati {len(frontier_indices)} pixel di frontiera grezzi .")

            # 5. Connected component clustering and noise suppression
            frontier_clusters = self.group_frontiers(frontier_indices, width, tolerance=2, min_size=8)

            print(f"[Matematico] Raggruppamento completato: trovate {len(frontier_clusters)} frontiere valide e sicure.")

            # 6. Global exploration termination check
            if len(frontier_clusters) == 0:
                return None, None

            # 7. Target point selection from clustered frontiers
            centroids = []

            for cluster in frontier_clusters:
                cluster_array = np.array(cluster)
                
                # Ring topology mitigation: select the median frontier cell along the cluster
                # boundary instead of the geometric mean to prevent target collapse into the robot footprint
                mid_index = len(cluster_array) // 2
                target_pixel = cluster_array[mid_index]

                # 8. Transform pixel index to global map frame in meters with half-cell centering
                real_x = origin_x + (target_pixel[0] * resolution) + (resolution / 2.0)
                real_y = origin_y + (target_pixel[1] * resolution) + (resolution / 2.0)

                centroids.append({
                    'x': real_x,
                    'y': real_y,
                    'size': len(cluster),
                    'px': target_pixel[0],
                    'py': target_pixel[1]
                })

            print(f"[Matematico] Calcolati {len(centroids)} centroidi in coordinate reali (metri).")

            # 9. Heuristic candidate scoring
            best_centroid = None
            best_score = -1.0

            for centroid in centroids:
                size = centroid['size']

                # Compute Euclidean distance from current robot pose to candidate target
                distance = math.sqrt((centroid['x'] - robot_x_in_map)**2 + (centroid['y'] - robot_y_in_map)**2)

                # Skip candidates within goal tolerance to avoid zero-displacement completions
                if distance < 0.21:
                    continue

                # Project candidate target backward along line of sight for safe clearance
                offset = 0
                
                if distance > offset:
                    ratio = (distance - offset) / distance
                    cx = robot_x_in_map + (centroid['x'] - robot_x_in_map) * ratio
                    cy = robot_y_in_map + (centroid['y'] - robot_y_in_map) * ratio
                else:
                    cx = centroid['x']
                    cy = centroid['y']
                
                safe_distance = math.sqrt((cx - robot_x_in_map)**2 + (cy - robot_y_in_map)**2)
                
                # Prevent zero-division singularity
                if safe_distance <= 0.0:
                    safe_distance = 0.1

                # Objective utility function: score = information_gain / travel_distance
                score = size / safe_distance

                # Apply exponential penalty decay based on historical failure count
                coord_key = (centroid['px'], centroid['py'])
                
                if coord_key in self.failed_goals:
                    failures = self.failed_goals[coord_key]
                    score = score / (2 ** failures)
                    print(f"[Matematico] ATTENZIONE: Penalizzato {coord_key}. Nuovo score: {score:.2f}")

                # Retain candidate maximizing utility score
                if score > best_score:
                    best_score = score
                    best_centroid = {'x': cx, 'y': cy, 'size': size, 'px': centroid['px'], 'py': centroid['py']}

            # 10. Safety verification and goal dispatch preparation
            if best_centroid is None:
                return None, None

            # Cache chosen pixel to register potential navigation failures
            self.last_target_pixel = (best_centroid['px'], best_centroid['py'])

            print(f"[Matematico] VINCITORE: X={best_centroid['x']:.2f}, Y={best_centroid['y']:.2f} (Score: {best_score:.2f})")
            
            return best_centroid['x'], best_centroid['y']