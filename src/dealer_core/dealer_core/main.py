#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import time
import math

from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist  
from nav_msgs.msg import Odometry
from card_dealer_msgs.msg import RoomConfig

from enum import Enum, auto
from typing import Dict, List

from .RoomCodeGenerator import create_unique_room
from .MongoDB_to_NETPIE import RoomCode_to_NETPIE


# ====== HELPER MATH FUNCTIONS ==============================================

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to yaw (radians)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """Normalize angle to be between -pi and pi."""
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

# Map Unicode symbols to LCD Node tokens
SYMBOL_TO_LCD = {
    "♠": "s/", "♥": "h/", "♦": "d/", "♣": "c/",
    "♤": "S/", "♡": "H/", "♢": "D/", "♧": "C/",
}

def format_code_for_lcd(room_code: str) -> str:
    lcd_tokens = []
    for char in room_code:
        token = SYMBOL_TO_LCD.get(char, char)
        lcd_tokens.append(token)
    return " ".join(lcd_tokens)


# ====== STATE ENUM ==========================================================

class DealerState(Enum):
    IDLE = auto()
    SETUP = auto()
    WAIT_CONFIG = auto()
    CALIBRATING_PLAYERS = auto()
    WAIT_START = auto()      
    INITIAL_DEAL = auto()
    DEALING_ROUND = auto()   # [ACTIVE] Main game loop
    WAIT_NEXT = auto()
    GAME_END = auto()
    ERROR = auto()


# ====== CARD DEALER CORE NODE ==============================================

class CardDealerCoreNode(Node):

    def __init__(self):
        super().__init__("card_dealer_core")

        qos = QoSProfile(depth=10)

        # --- Publishers --- #
        self.dealer_state_pub = self.create_publisher(String, "/dealer/state", qos)
        self.dealer_event_pub = self.create_publisher(String, "/dealer/event", qos)
        self.lcd_line1 = self.create_publisher(String, "/lcd/display/line1", qos)
        self.lcd_line2 = self.create_publisher(String, "/lcd/display/line2", qos)
        
        # Card Count Publisher
        self.cards_left_pub = self.create_publisher(Int32, "/core/card_left", qos)
        
        # Hardware Control
        self.rotator_pub = self.create_publisher(Float32, '/rotator/angle', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.card_spreader_pub = self.create_publisher(String, '/card_spreader/cmd', 10)

        # --- Subscribers --- #
        self.mqtt_room_config_sub = self.create_subscription(
            RoomConfig, "/mqtt/room_config", self.room_config_callback, qos
        )
        self.button_pressed_sub = self.create_subscription(
            Bool, '/lcd/button', self.button_pressed_callback, qos
        )
        
        # IR Angle (for Calibration)
        self.ir_angle_sub = self.create_subscription(
            Float32, '/ir_reader/player_angle', self.ir_angle_callback, 10
        )
        
        # IR Detection (for Hand Waving)
        self.ir_detection_sub = self.create_subscription(
            Bool, '/ir_reader/player_detection', self.ir_detection_callback, 10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos
        )
        self.rotator_status_sub = self.create_subscription(
            Bool, '/rotator/status', self.rotator_status_callback, 10
        )

        # --- Variables --- #
        self.state = DealerState.IDLE
        self.previous_state = None

        self.is_button_pressed = False
        self.room_code: str = ""
        self.room_status: str = "pending"  
        self.num_players: int = 0
        self.card_gain: int = 1 # Default 1
        self.starting_card: int = 0
        
        # Card Tracking Variables
        self.deck_size = 52     
        self.cards_left = 52    
        
        self.players_data = [] 
        
        # [UPDATED] Map Name (String) to Angle (Float) instead of Order (Int)
        self.player_angle_map: Dict[str, float] = {}

        # Odom tracking
        self.current_yaw = 0.0

        # Calibration Vars
        self.collected_angles: List[float] = []
        self.rotate_start_time = None
        self.rotate_count = 0
        self.max_count = 3
        
        # Initial Deal Vars
        self.deal_player_order_list = []
        self.current_deal_idx = 0
        self.current_round = 1 
        self.deal_phase = "ROTATING" 
        self.rotation_sent = False
        self.rotator_ready = False
        self.last_deal_time = None
        
        # Round Dealing Vars
        self.round_substate = "WAIT_BUTTON" 
        self.hand_wait_start_time = None
        self.is_hand_detected = False
        self.cards_dealt_in_turn = 0 
        self.pause_lcd_updated = False

        # --- Loop --- #
        self.timer = self.create_timer(0.1, self.state_machine_step)


    # ====== CALLBACKS ==============================================

    def ir_detection_callback(self, msg: Bool):
        self.is_hand_detected = msg.data

    def rotator_status_callback(self, msg: Bool):
        if msg.data == True:
            self.rotator_ready = True

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def button_pressed_callback(self, msg: Bool):
        if msg.data == True:
            self.get_logger().info("Button Pressed")
            self.is_button_pressed = True
    
    def room_config_callback(self, msg: RoomConfig):
        if msg.room_code != self.room_code: return
        
        self.room_code = msg.room_code
        self.room_status = msg.room_status
        self.num_players = msg.num_players
        self.card_gain = msg.card_gain
        self.starting_card = msg.starting_card
        
        self.players_data = []
        for p in msg.players:
            self.players_data.append({
                "name": p.name, 
                "order": p.order, 
                "skipped": p.skipped
            })

    def ir_angle_callback(self, msg: Float32):
        if self.state == DealerState.CALIBRATING_PLAYERS:
            angle = msg.data
            is_duplicate = False
            for existing_angle in self.collected_angles:
                diff = abs(existing_angle - angle)
                if diff > 180: diff = 360 - diff 
                if diff < 8.0:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                self.collected_angles.append(angle)
                found = len(self.collected_angles)
                self.publish_lcd_display(2, f"Found: {found}/{self.num_players}")

    # ====== UTILS ==============================================

    def stop_robot(self):
        stop_cmd = Twist()
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        
    def publish_cards_left(self):
        msg = Int32()
        msg.data = self.cards_left
        self.cards_left_pub.publish(msg)

    def map_players_to_angles(self) -> bool:
        if len(self.collected_angles) != self.num_players:
            return False
        
        # Sort angles (physical locations from 0 to 360)
        final_angles = sorted(self.collected_angles)
        
        # Sort players by their INITIAL order (at time of calibration)
        sorted_players = sorted(self.players_data, key=lambda x: x['order'])

        self.player_angle_map = {}
        self.get_logger().info("--- Player Mapping (NAME -> ANGLE) ---")
        for i in range(self.num_players):
            name = sorted_players[i]['name'] # Key is Name, not Order
            angle = final_angles[i]
            
            # [UPDATED] Store by Name
            self.player_angle_map[name] = angle
            self.get_logger().info(f"  {name} -> {angle:.2f} deg")
        return True

    def publish_state(self):
        state_msg = String()
        state_msg.data = self.state.name
        self.dealer_state_pub.publish(state_msg)

    def publish_lcd_display(self, line: int, message: str):
        msg = String()
        msg.data = message
        if line == 1: self.lcd_line1.publish(msg)
        elif line == 2: self.lcd_line2.publish(msg)

    def on_enter_state(self, new_state):
        self.publish_state() 
        self.get_logger().info(f"--- Entering State: {new_state.name} ---")

        if new_state == DealerState.IDLE:
            self.stop_robot()
            self.publish_lcd_display(1, "PRESSED TO")
            self.publish_lcd_display(2, "START")

        elif new_state == DealerState.SETUP:
            self.publish_lcd_display(1, "Creating Room...")
            self.publish_lcd_display(2, "Please Wait")

        elif new_state == DealerState.WAIT_CONFIG:
            self.publish_lcd_display(1, "Room Code:")
            formatted_code = format_code_for_lcd(self.room_code)
            self.publish_lcd_display(2, formatted_code)

        elif new_state == DealerState.CALIBRATING_PLAYERS:
            self.publish_lcd_display(1, "Calibrating...")
            self.publish_lcd_display(2, f"Found: 0/{self.num_players}")
            
            self.collected_angles = []
            self.rotate_start_time = self.get_clock().now()
            self.rotate_count = 0
            
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
        
        elif new_state == DealerState.WAIT_START:
            self.publish_lcd_display(1, "Calibration OK")
            self.publish_lcd_display(2, "Press to Start")
            self.stop_robot() 

        elif new_state == DealerState.INITIAL_DEAL:
            self.publish_lcd_display(1, "Initial Deal")
            self.publish_lcd_display(2, "Starting...")
            
            self.cards_left = self.deck_size
            self.publish_cards_left()
            
            self.deal_player_order_list = sorted(self.players_data, key=lambda x: x['order'])
            
            self.current_deal_idx = 0      
            self.current_round = 1         
            self.deal_phase = "ROTATING"
            self.rotation_sent = False
            self.rotator_ready = False 
            self.last_deal_time = self.get_clock().now()
        
        elif new_state == DealerState.DEALING_ROUND:
            self.publish_lcd_display(1, "Round Start")
            self.publish_lcd_display(2, "Press Button...")
            self.round_substate = "WAIT_BUTTON"
            self.stop_robot()

    # ====== STATE MACHINE STEP ==============================================
    def state_machine_step(self):
        if self.state != self.previous_state:
            self.on_enter_state(self.state)
            self.previous_state = self.state

        # --- IDLE ---
        if self.state == DealerState.IDLE:
            if self.is_button_pressed:  
                self.is_button_pressed = False
                self.state = DealerState.SETUP

        # --- SETUP ---
        elif self.state == DealerState.SETUP:
            try:
                new_room = create_unique_room() 
                self.room_code = new_room["room_code"]
                self.get_logger().info(f"Room Created: {self.room_code}")
                RoomCode_to_NETPIE(self.room_code)
                self.state = DealerState.WAIT_CONFIG
            except Exception as e:
                self.get_logger().error(f"Setup Error: {e}")
                self.state = DealerState.ERROR

        # --- WAIT CONFIG ---
        elif self.state == DealerState.WAIT_CONFIG:
            if self.room_status == "running":
                self.state = DealerState.CALIBRATING_PLAYERS
            elif self.room_status == "cancelled":
                self.state = DealerState.IDLE

        # --- CALIBRATION ---
        elif self.state == DealerState.CALIBRATING_PLAYERS:
            now = self.get_clock().now()
            cycle_time = (now - self.rotate_start_time).nanoseconds / 1e9
            
            twist = Twist()
            if cycle_time < 15.0:
                twist.angular.z = 0.5
            elif cycle_time < 30.0:
                twist.angular.z = -0.5
            else:
                self.rotate_start_time = now
                self.rotate_count += 1
                return 

            self.cmd_vel_pub.publish(twist)

            if len(self.collected_angles) >= self.num_players:
                self.stop_robot()
                if self.map_players_to_angles():
                    self.get_logger().info("Calibration Success!")
                    self.state = DealerState.WAIT_START 
                else:
                    self.state = DealerState.ERROR

            if self.rotate_count > self.max_count:
                self.stop_robot()
                self.publish_lcd_display(1, "Timeout Error")
                self.state = DealerState.ERROR
        
        # --- WAIT START ---
        elif self.state == DealerState.WAIT_START:
            if self.is_button_pressed:
                self.is_button_pressed = False
                self.get_logger().info("User Confirmed. Starting Game.")
                self.state = DealerState.INITIAL_DEAL

        # --- INITIAL DEAL ---
        elif self.state == DealerState.INITIAL_DEAL:
            if self.current_round > self.starting_card:
                self.get_logger().info("Initial Deal Complete!")
                self.state = DealerState.DEALING_ROUND
                return

            p_data = self.deal_player_order_list[self.current_deal_idx]
            current_name = p_data['name']
            
            # [UPDATED] Lookup by NAME
            target_deg = self.player_angle_map.get(current_name)
            
            if target_deg is None:
                self.get_logger().warn(f"Name {current_name} not found in map!")
                self.current_deal_idx += 1
                return

            if self.deal_phase == "ROTATING":
                if not self.rotation_sent:
                    self.rotator_ready = False 
                    msg = Float32()
                    msg.data = float(target_deg)
                    self.rotator_pub.publish(msg)
                    self.rotation_sent = True
                    self.get_logger().info(f"Rotating to {current_name} ({target_deg:.1f})...")
                
                if self.rotator_ready:
                    self.deal_phase = "DEALING"
                    self.last_deal_time = self.get_clock().now() 
                    self.publish_lcd_display(2, f"Deal {current_name}: {self.current_round}/{self.starting_card}")

            elif self.deal_phase == "DEALING":
                now = self.get_clock().now()
                elapsed = (now - self.last_deal_time).nanoseconds / 1e9
                if elapsed > 1.5:
                    self.card_spreader_pub.publish(String(data="START"))
                    
                    self.cards_left -= 1
                    self.publish_cards_left()
                    
                    self.get_logger().info(f"Dealt card {self.current_round} to {current_name}")
                    
                    self.current_deal_idx += 1
                    if self.current_deal_idx >= len(self.deal_player_order_list):
                        self.current_deal_idx = 0
                        self.current_round += 1
                    
                    self.deal_phase = "ROTATING"
                    self.rotation_sent = False

        # --- [ACTIVE] DEALING ROUND ---
        elif self.state == DealerState.DEALING_ROUND:

            if self.room_status == "paused":
                self.stop_robot()
                if not self.pause_lcd_updated:
                    self.publish_lcd_display(1, "GAME PAUSED")
                    self.publish_lcd_display(2, "Wait Resume...")
                    self.pause_lcd_updated = True
                return
            else:
                self.pause_lcd_updated = False

            if self.round_substate == "WAIT_BUTTON":
                if self.is_button_pressed:
                    self.is_button_pressed = False
                    self.deal_player_order_list = sorted(self.players_data, key=lambda x: x['order'])
                    self.current_deal_idx = 0
                    self.round_substate = "ROTATING"
                    self.get_logger().info("Round Start! Moving to first player.")

            elif self.round_substate == "ROTATING":
                if self.current_deal_idx >= len(self.deal_player_order_list):
                    self.get_logger().info("Round Finished. Waiting for next button press.")
                    self.publish_lcd_display(1, "Round Done")
                    self.publish_lcd_display(2, "Press to Next")
                    self.round_substate = "WAIT_BUTTON"
                    return

                p_data = self.deal_player_order_list[self.current_deal_idx]
                p_order = p_data['order']
                p_name = p_data['name']

                if p_data.get('skipped', False):
                    self.get_logger().info(f"Skipping {p_name} (skipped=true)")
                    self.current_deal_idx += 1
                    return

                # [UPDATED] Lookup by NAME
                target_deg = self.player_angle_map.get(p_name)
                
                if target_deg is None:
                    self.get_logger().warn(f"Name {p_name} not found in angle map!")
                    self.current_deal_idx += 1
                    return

                if not self.rotation_sent:
                    self.rotator_ready = False
                    msg = Float32()
                    msg.data = float(target_deg)
                    self.rotator_pub.publish(msg)
                    self.rotation_sent = True
                    self.publish_lcd_display(1, f"Finding: {p_name}")
                    self.publish_lcd_display(2, "Moving...")

                if self.rotator_ready:
                    self.get_logger().info(f"Arrived at {p_name}. Waiting for hand...")
                    self.publish_lcd_display(1, f"{p_name}")
                    self.publish_lcd_display(2, "Show Hand...")
                    
                    self.round_substate = "WAITING_HAND"
                    self.hand_wait_start_time = self.get_clock().now()
                    self.rotation_sent = False
                    self.is_hand_detected = False 

            elif self.round_substate == "WAITING_HAND":
                if self.is_hand_detected:
                    self.get_logger().info("Hand Detected! Dealing...")
                    self.round_substate = "DEALING"
                    self.cards_dealt_in_turn = 0
                    self.last_deal_time = self.get_clock().now()
                
                else:
                    now = self.get_clock().now()
                    elapsed = (now - self.hand_wait_start_time).nanoseconds / 1e9
                    
                    if elapsed > 3.0:
                        self.get_logger().info(f"Timeout (3s). Skipping Player.")
                        self.publish_lcd_display(2, "Skipped (Timeout)")
                        time.sleep(0.5) 
                        self.current_deal_idx += 1
                        self.round_substate = "ROTATING"

            elif self.round_substate == "DEALING":
                now = self.get_clock().now()
                elapsed = (now - self.last_deal_time).nanoseconds / 1e9

                if elapsed > 1.5:
                    if self.cards_dealt_in_turn < self.card_gain:
                        self.card_spreader_pub.publish(String(data="START"))
                        
                        self.cards_left -= 1
                        self.publish_cards_left()
                        
                        self.cards_dealt_in_turn += 1
                        self.last_deal_time = now
                        
                        p_name = self.deal_player_order_list[self.current_deal_idx]['name']
                        self.publish_lcd_display(2, f"Dealing: {self.cards_dealt_in_turn}/{self.card_gain}")
                        self.get_logger().info(f"Dealt card {self.cards_dealt_in_turn} to {p_name}")
                    else:
                        self.current_deal_idx += 1
                        self.round_substate = "ROTATING"

        # --- ERROR ---
        elif self.state == DealerState.ERROR:
            if self.is_button_pressed:
                self.is_button_pressed = False
                self.state = DealerState.IDLE
    
def main(args=None):
    rclpy.init(args=args)
    node = CardDealerCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()