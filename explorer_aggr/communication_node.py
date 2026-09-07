import rclpy
from rclpy.node import Node
# TF2 buffer and transform listener for tracking robot pose in map frame
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
# Standard ROS 2 occupancy grid message type
from nav_msgs.msg import OccupancyGrid
# Nav2 action client and goal specification
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
# Frontier detection and scoring module
from explorer_aggr.nav2_goal_setter import GoalSetter

from rclpy.parameter import Parameter

import math
import numpy as np


from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan




class CommunicationNode(Node):
    def __init__(self):
        
        super().__init__('communication_node')

        # Subscribe to /map topic with a QoS queue depth of 10
        self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)

        # Action client targeting Nav2 navigate_to_pose action server
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        # Enforce simulation clock synchronization
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])



        # Navigation state machine flags
        self.is_navigating = False
        self.exploration_completed = False
        
        # Initial robot pose cache for Return-to-Home (RTH) routine
        self.home_x = None
        self.home_y = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Active goal coordinates
        self.current_goal_x = None
        self.current_goal_y = None

        # Instantiate frontier goal evaluator
        self.goal_setter = GoalSetter()

        self.get_logger().info("Communication Node avviato! Pronto per l'esplorazione.")

        # Dynamic stall detection state variables (20s stationary timeout)
        self.stuck_check_timer = None
        self.last_check_x = None
        self.last_check_y = None
        self.seconds_stuck = 0
        self.current_goal_handle = None

        self.send_goal_future = None
        self.get_result_future = None

        self.maps_received = 0

        # Trajectory starting pose for false-positive arrival validation
        self.start_pose_x = None
        self.start_pose_y = None

        self.recovery_attempts = 0
        self.max_recoveries = 2  # Maximum recovery attempts prior to RTH abort



        # --- CUSTOM LIDAR RECOVERY SUBSYSTEM ---
        # Direct velocity command publisher for open-loop recovery maneuvers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Laser scan subscription for obstacle clearance evaluation
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # State machine tracking custom recovery execution
        self.is_in_custom_recovery = False
        self.recovery_phase = 0      # Phase 0: LiDAR analysis, Phase 1: Rotation, Phase 2: Forward push
        self.target_escape_yaw = 0.0 # Computed optimal escape heading
        self.recovery_start_x = 0.0  # Displacement reference coordinate X
        self.recovery_start_y = 0.0  # Displacement reference coordinate Y
        
        # Periodic control loop timer executing recovery trajectory (10 Hz)
        self.custom_recovery_timer = self.create_timer(0.1, self.custom_recovery_loop)


        # Consecutive recalculation failures counter while stationary
        self.stuck_recalculations = 0

        # --- RECENT GOAL HISTORY TRACKER ---
        # Cache recent target poses to prevent cyclic goal oscillation
        self.recent_goals_history = [] 
        self.ping_pong_counter = 0


        # --- NAVIGATION WATCHDOG TIMER ---
        self.nav_watchdog_timer = None
        self.max_nav_duration = 70.0    # Maximum execution window permitted per goal (seconds)

        





    def map_callback(self, msg):
            
            # Cache latest occupancy grid message for recovery analysis
            self.last_map_msg = msg


            # Guard clause: ignore incoming maps while navigating or when exploration is complete
            if self.exploration_completed or self.is_navigating:
                return
            
            # Discard initial frames to ensure SLAM map stabilization
            self.maps_received += 1

            # Retrieve current robot pose from TF tree
            rx, ry, ryaw = self.get_robot_pose()
            
            # Await valid TF transform availability
            if rx is None or ry is None:
                return
            
                
            # Cache initial valid pose as origin benchmark for return-to-home
            if self.home_x is None and self.home_y is None:
                self.home_x = rx
                self.home_y = ry
                self.get_logger().info(f"Posizione di base memorizzata: X={rx:.2f}, Y={ry:.2f}")
                
            # Compute new exploration frontier
            self.get_logger().info("Robot libero! Inizio calcolo della nuova frontiera...")
            
            # Dispatch map and current pose to geometric frontier selector
            goal_x, goal_y = self.goal_setter.calculate_best_frontier(msg, rx, ry)
            
            # Hold if map resolution or dimension checks indicate pending data
            if goal_x == "WAIT":
                return

            # Check exploration completion or initial SLAM warm-up state
            if goal_x is None or goal_y is None:
                # Prevent premature termination during initial mapping warm-up
                if self.maps_received < 5:
                    self.get_logger().info(f" sono alla (Mappa {self.maps_received}), Siccome non vedo nessuna frontiera possibile penso che sia lo slam che deve ancora caricare")
                    return
                
                # --- RECOVERY TRIGGER ---
                if self.recovery_attempts < self.max_recoveries:
                    self.recovery_attempts += 1
                    self.get_logger().warn(f"Nessuna frontiera! Tentativo di Recovery {self.recovery_attempts}/{self.max_recoveries}: Eseguo un Blind Push in avanti di 30cm...")
                    

                    # Lock map callback and trigger custom LiDAR escape routine
                    self.is_navigating = True 
                    
                    self.is_in_custom_recovery = True
                    self.recovery_phase = 0  # Phase 0: Analyze scan and determine escape heading
                    return

                # Terminal state: return to base when no valid frontiers remain
                self.get_logger().info("NESSUNA FRONTIERA TROVATA! Esplorazione finita. Torno a casa...")
                self.exploration_completed = True
                
                self.current_goal_x = self.home_x
                self.current_goal_y = self.home_y

                # Dispatch return-to-home goal to Nav2
                self.send_nav2_goal(self.home_x, self.home_y)
                return
                
            # Dispatch validated frontier goal to Nav2
            self.get_logger().info(f"Miglior frontiera trovata: X={goal_x:.2f}, Y={goal_y:.2f}")
            
            # Update target state
            self.current_goal_x = goal_x
            self.current_goal_y = goal_y
            
            # Record departure coordinates for false-success validation
            self.start_pose_x = rx
            self.start_pose_y = ry

            # Reset recovery counter upon identifying an actionable frontier
            self.recovery_attempts = 0





            # --- ANTI PING-PONG VALIDATION ---
            is_ping_pong = False
            
            # Verify minimum Euclidean distance against recently commanded targets
            for past_x, past_y in self.recent_goals_history:
                dist = math.sqrt((goal_x - past_x)**2 + (goal_y - past_y)**2)
                if dist < 0.20: # Flag candidate if within 0.20m of recent target
                    is_ping_pong = True
                    break
            
            if is_ping_pong:
                self.ping_pong_counter += 1
                self.get_logger().warn(f"Déjà vu rilevato (Rimbalzo {self.ping_pong_counter}/3). Sto puntando una vecchia zona!")
                
                # Trigger custom LiDAR escape if cyclic oscillation threshold is reached
                if self.ping_pong_counter >= 3:
                    self.get_logger().error("PING-PONG CRITICO! Mappa in ritardo. SCATTA LA RECOVERY LIDAR!")
                    self.ping_pong_counter = 0
                    self.recent_goals_history.clear()
                    
                    self.is_navigating = True 
                    self.is_in_custom_recovery = True
                    self.recovery_phase = 0
                    return # Bypass Nav2 dispatch
            else:
                # Append unique target to history buffer
                self.ping_pong_counter = 0
                self.recent_goals_history.append((goal_x, goal_y))
                
                # Maintain sliding window size of 4 entries
                if len(self.recent_goals_history) > 4:
                    self.recent_goals_history.pop(0)



            # Set navigation lock and dispatch goal
            self.is_navigating = True
            self.send_nav2_goal(goal_x, goal_y)





    def get_robot_pose(self):
        """
        Lookup the latest transform from map to base_link via TF2 buffer.
        """
        try:
            # Query the latest available transform from map to base_link
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Extract 2D translation and calculate yaw from quaternion
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
            
        except TransformException as ex:
            # Catch uninitialized or unavailable transforms without raising
            self.get_logger().warn(f"Posizione non ancora disponibile: {ex}")
            return None, None, None
        



    def send_nav2_goal(self, x, y):
        """
        Construct and dispatch a NavigateToPose action goal to Nav2.
        """
        # Block until Nav2 action server is available
        self.nav_to_pose_client.wait_for_server()

        # Build NavigateToPose action goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Populate target coordinates
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Identity orientation (heading handled dynamically by path planner)
        goal_msg.pose.pose.orientation.w = 1.0 

        self.get_logger().info(f"Invio goal a Nav2: X={x:.2f}, Y={y:.2f}")

        # Send goal asynchronously and bind response callback
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)


        # Arm the 70-second execution watchdog timer
        if self.nav_watchdog_timer is not None:
            self.nav_watchdog_timer.cancel()

        self.nav_watchdog_timer = self.create_timer(
            self.max_nav_duration, 
            self.navigation_timeout_callback
        )




    def goal_response_callback(self, future):
        """
        Evaluate Nav2 goal acceptance or rejection.
        """
        self.current_goal_handle = future.result()
        goal_handle = self.current_goal_handle

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 ha rifiutato il goal! (Forse è irraggiungibile)")
            # Release navigation lock upon rejection
            self.is_navigating = False
            return

        self.get_logger().info("Nav2 ha accettato il goal. In viaggio...")

        # Initialize dynamic stall detection check
        rx, ry, _ = self.get_robot_pose()
        self.last_check_x = rx
        self.last_check_y = ry
        self.seconds_stuck = 0
        
        # Schedule periodic stall inspection every 2.0 seconds
        self.stuck_check_timer = self.create_timer(2.0, self.check_if_stuck_callback)


        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)




    def get_result_callback(self, future):
        """
        Handle navigation completion, abort, or false-positive arrival.
        """
        # Disarm stall monitor
        if self.stuck_check_timer is not None:
            self.stuck_check_timer.cancel()
            self.stuck_check_timer = None 
            self.seconds_stuck = 0


        # Disarm watchdog timer
        if self.nav_watchdog_timer is not None:
            self.nav_watchdog_timer.cancel()
            self.nav_watchdog_timer = None



        # Evaluate Nav2 status: 4 corresponds to SUCCEEDED
        status = future.result().status
        
        if status == 4:
            # Validate physical displacement to filter tolerance-triggered false positives
            rx, ry, _ = self.get_robot_pose()
            
            if rx is not None and self.start_pose_x is not None:

                dist_moved = math.sqrt((rx - self.start_pose_x)**2 + (ry - self.start_pose_y)**2)
            
                if dist_moved < 0.10: # Flag goals completed with under 10cm displacement
                    self.get_logger().warn(f"ANTI-CHEAT: Nav2 ha dichiarato successo ma si è mosso solo di {dist_moved:.2f}m. È un falso bersaglio!")
                    # Penalize coordinates to prevent repeated selection
                    self.goal_setter.register_failure(self.current_goal_x, self.current_goal_y)
                else:
                    self.get_logger().info("Obiettivo raggiunto con successo ! Calcolo nuova rotta...")
            
            self.is_navigating = False
            
        else:
            self.get_logger().warn("Navigazione fallita o interrotta durante il tragitto.")
            
            # Penalize coordinates on navigation failure
            if self.current_goal_x is not None and self.current_goal_y is not None:
                self.goal_setter.register_failure(self.current_goal_x, self.current_goal_y)
                self.get_logger().info(f"Penalizzata la coordinata: X={self.current_goal_x:.2f}, Y={self.current_goal_y:.2f}")
            

            # Release lock only if custom recovery is inactive
            if not getattr(self, 'is_in_custom_recovery', False):
                self.is_navigating = False
            else:
                self.get_logger().info("Nav2 spento con successo. Il pilota automatico Lidar ha i comandi!")
    


    def check_if_stuck_callback(self):
        """
        Inspect physical displacement every 2 seconds and cancel trajectory after 20s stall.
        """
        rx, ry, _ = self.get_robot_pose()
        
        if rx is None or self.last_check_x is None:
            return

        # Compute Euclidean displacement over the last 2 seconds
        dist = math.sqrt((rx - self.last_check_x)**2 + (ry - self.last_check_y)**2)
        
        if dist < 0.05:
            # Increment stall duration counter if displacement is under 5cm
            self.seconds_stuck += 2
            
            if self.seconds_stuck % 4 == 0:
                self.get_logger().info(f"Monitoraggio: Il robot sembra fermo da {self.seconds_stuck}/20 secondi...")
        else:
            # Reset stall tracking variables upon confirmed movement
            self.seconds_stuck = 0
            self.last_check_x = rx
            self.last_check_y = ry
            self.stuck_recalculations = 0

        # Trigger cancellation if stationary threshold (20s) is exceeded
        if self.seconds_stuck >= 20:
            self.get_logger().warn("TIMEOUT DINAMICO! Il robot è incastrato (non avanza da 20s). Annullamento in corso...")
            
            # Disarm inspection timer
            if self.stuck_check_timer is not None:
                self.stuck_check_timer.cancel()
                self.stuck_check_timer = None
                self.seconds_stuck = 0

            # Cancel active Nav2 goal
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self.stuck_recalculations += 1
            
            if self.stuck_recalculations >= 2:
                self.get_logger().error(f"Incastro critico: {self.stuck_recalculations} fallimenti! SCATTA LA RECOVERY LIDAR.")
                self.stuck_recalculations = 0
                
                # Engage LiDAR recovery routine
                self.is_in_custom_recovery = True
                self.recovery_phase = 0
            else:
                self.get_logger().warn(f"Attendo il nuovo ricalcolo del Matematico (Tentativo {self.stuck_recalculations}/2)...")




    def scan_callback(self, msg):
        """
        Analyze LiDAR ranges and OccupancyGrid to determine an optimal escape heading.
        Applies a centering algorithm over valid angular sectors to avoid doorway edges.
        """

        # Guard clause: process scans only during active recovery
        if not self.is_in_custom_recovery:
            return

        # Emergency collision brake during Phase 2 forward push
        if self.recovery_phase == 2:
            ranges_brake = np.array(msg.ranges)
            ranges_brake = np.nan_to_num(ranges_brake, posinf=10.0, neginf=0.0)
            
            # Extract 30-degree frontal cone centered on robot heading
            front_ranges = np.concatenate((ranges_brake[-15:], ranges_brake[:15]))
            min_front_dist = np.min(front_ranges)
            
            # Abort forward push if obstacle proximity violates safety threshold (< 0.22m)
            if min_front_dist < 0.22:
                self.get_logger().error(f"ANTI-COLLISIONE: Ostacolo a {min_front_dist:.2f}m! Arresto la spinta.")
                self.stop_custom_recovery()
            return

        # Defer scan analysis during Phase 1 rotational alignment
        if self.recovery_phase == 1:
            return

        # Query robot pose and heading in global map frame
        rx, ry, current_yaw = self.get_robot_pose()
        if rx is None or current_yaw is None:
            return

        # Check occupancy grid availability
        if not hasattr(self, 'last_map_msg') or self.last_map_msg is None:
            self.get_logger().warn("[CUSTOM RECOVERY] Nessuna mappa in memoria. Il robot non vedrà l'ignoto.")
            map_available = False
        else:
            map_available = True
            m_info = self.last_map_msg.info
            # Reshape 1D flat occupancy array into 2D grid matrix
            m_data = np.array(self.last_map_msg.data, dtype=np.int8).reshape((m_info.height, m_info.width))

        # Condition laser ranges (substitute inf with 10.0m and NaN with 0.0m)
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, posinf=10.0, neginf=0.0)
        
        ray_results = []

        # Decimate rays (step = 3) for efficient array processing
        step = 3
        indices = np.arange(0, len(ranges), step)
        r_vals = ranges[indices]
        
        # Filter rays with obstacle clearance under 0.40m
        valid_mask = r_vals >= 0.40
        valid_indices = indices[valid_mask]
        valid_r = r_vals[valid_mask]

        if len(valid_indices) > 0:
            
            # Vectorized angular conversion to global map frame
            relative_angles = msg.angle_min + (valid_indices * msg.angle_increment)
            absolute_yaws = current_yaw + relative_angles
            
            # Assign base score proportional to measured range clearance
            scores = valid_r.copy()

            # Cross-reference ray endpoints against occupancy grid
            if map_available:
                check_distances = np.minimum(valid_r, 3.0)
                
                # Project ray endpoints trigonometrically
                end_x = rx + (check_distances * np.cos(absolute_yaws))
                end_y = ry + (check_distances * np.sin(absolute_yaws))
                
                # Convert metric coordinates to map pixel indices
                px = ((end_x - m_info.origin.position.x) / m_info.resolution).astype(int)
                py = ((end_y - m_info.origin.position.y) / m_info.resolution).astype(int)
                
                # Evaluate boundary bounds
                in_bounds = (px >= 0) & (px < m_info.width) & (py >= 0) & (py < m_info.height)
                
                pixel_values = np.zeros_like(scores, dtype=np.int8)
                pixel_values[in_bounds] = m_data[py[in_bounds], px[in_bounds]]
                
                # Reward unexplored space (-1) and penalize proximity to inflated obstacles (> 50)
                scores[in_bounds & (pixel_values == -1)] += 100.0
                scores[in_bounds & (pixel_values > 50)] -= 50.0

            ray_results = list(zip(absolute_yaws, scores))

        # --- SECTOR CENTERING: COMPUTE MEDIAN ANGLE OF WIDEST CLEARANCE SECTOR ---
        best_angle = None
        best_score = -999.0

        if ray_results: 
            
            max_score = max(res[1] for res in ray_results)
            best_score = max_score

            # Identify contiguous ray clusters reaching maximum score
            best_cluster = []
            current_cluster = []

            for angle, score in ray_results:
                if abs(score - max_score) < 0.5: 
                    current_cluster.append(angle)
                else:
                    if len(current_cluster) > len(best_cluster):
                        best_cluster = current_cluster
                    current_cluster = [] 

            if len(current_cluster) > len(best_cluster):
                best_cluster = current_cluster

            # Select median heading to center escape trajectory through the opening
            if best_cluster:
                mid_index = len(best_cluster) // 2
                best_angle = best_cluster[mid_index]

        # Fallback maneuver if all rays violate 0.40m clearance
        if best_angle is None:
            self.get_logger().error("[CUSTOM RECOVERY] Nessuna via libera a >40cm. Provo un micro-spin per sbloccare i cingoli.")
            best_angle = current_yaw + 0.5

        # Normalize escape heading to [-pi, pi]
        self.target_escape_yaw = math.atan2(math.sin(best_angle), math.cos(best_angle))
        
        self.get_logger().warn(f"[CUSTOM RECOVERY] Via centrata calcolata! (Score tattico: {best_score:.1f}). Avvio motori!")
        
        # Record trajectory origin and transition to Phase 1 (alignment rotation)
        self.recovery_start_x = rx
        self.recovery_start_y = ry
        self.recovery_phase = 1











    def custom_recovery_loop(self):
        """
        Execute open-loop recovery maneuvers: in-place rotation (Phase 1) and forward push (Phase 2).
        """
        # Guard clause: execute only during active recovery state
        if not self.is_in_custom_recovery or self.recovery_phase == 0:
            return

        msg = Twist()
        rx, ry, current_yaw = self.get_robot_pose()
        
        if current_yaw is None:
            return

        # --- PHASE 1: ALIGNMENT ROTATION TOWARD ESCAPE HEADING ---
        if self.recovery_phase == 1:

            # Compute angular heading error normalized to [-pi, pi]
            error = math.atan2(math.sin(self.target_escape_yaw - current_yaw), 
                               math.cos(self.target_escape_yaw - current_yaw))
            

            if abs(error) > 0.1: # Heading tolerance threshold (~5 degrees)
                # Pure rotation at 0.5 rad/s
                msg.angular.z = 0.5 if error > 0 else -0.5
            else:
                # Alignment achieved: transition to forward push
                msg.angular.z = 0.0
                self.get_logger().info("[CUSTOM RECOVERY] Allineato. Inizio avanzamento...")
                self.recovery_phase = 2
                self.recovery_start_x = rx
                self.recovery_start_y = ry

        # --- PHASE 2: FORWARD CLEARANCE PUSH ---
        elif self.recovery_phase == 2:
            # Measure cumulative displacement from push origin
            dist_pushed = math.sqrt((rx - self.recovery_start_x)**2 + (ry - self.recovery_start_y)**2)
            
            if dist_pushed < 0.40: # Execute 0.40m displacement
                msg.linear.x = 0.15 # Safe cruise speed
            else:
                self.get_logger().info("[CUSTOM RECOVERY] Manovra completata con successo.")
                self.stop_custom_recovery()
                return

        self.cmd_vel_pub.publish(msg)

    def stop_custom_recovery(self):
        """Halt motors, reset state machine, and yield control back to the planner."""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.is_in_custom_recovery = False
        self.is_navigating = False # Release navigation lock for map callback re-trigger
        self.recovery_phase = 0



    def navigation_timeout_callback(self):
        """Abort navigation when maximum execution window expires."""
        self.get_logger().error(f"TIMEOUT! Il robot non ha raggiunto il goal in {self.max_nav_duration}s. Annullamento in corso...")

        # Disarm watchdog timer
        if self.nav_watchdog_timer is not None:
            self.nav_watchdog_timer.cancel()
            self.nav_watchdog_timer = None

        # Cancel active Nav2 goal
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().warn("Goal Nav2 annullato dal Watchdog.")

        # Release navigation lock to allow replanning
        self.is_navigating = False





def main(args=None):
    # Initialize ROS 2 execution context
    rclpy.init(args=args)
    
    # Instantiate node
    node = CommunicationNode()
    
    # Spin node event loop
    rclpy.spin(node)
    
    # Destroy node and shutdown client library on termination
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()