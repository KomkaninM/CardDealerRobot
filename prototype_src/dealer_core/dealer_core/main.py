#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import time
import math

from std_msgs.msg import String, Float32, Int32, Bool
from card_dealer_msgs.msg import RoomConfig

from enum import Enum, auto
from typing import Dict, List

from RoomCodeGenerator import create_unique_room
from MongoDB_to_NETPIE import RoomCode_to_NETPIE


# ====== STATE ENUM ==========================================================

class DealerState(Enum):
    IDLE = auto()
    SETUP = auto()
    WAIT_CONFIG = auto()
    CALIBRATING_PLAYERS = auto()
    WAIT_START = auto()
    DEALING_ROUND = auto()
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
        
        # [ADDED] Publisher to control the rotator
        self.rotator_pub = self.create_publisher(Float32, '/rotator/angle', 10)

        # --- Subscribers --- #
        self.mqtt_room_config_sub = self.create_subscription(
            RoomConfig, 
            "/mqtt/room_config", 
            self.room_config_callback, 
            qos
        )

        self.button_pressed_sub = self.create_subscription(
            Bool, 
            '/lcd/button', 
            self.button_pressed_callback,
            qos
        )

        # [ADDED] Subscriber to listen for detected IR angles
        self.ir_angle_sub = self.create_subscription(
            Float32, 
            '/ir_reader/player_angle', 
            self.ir_angle_callback, 
            10
        )

        # ---Variable Declaration --- #
        self.state = DealerState.IDLE
        self.previous_state = None

        #LCD variables
        self.is_button_pressed = False

        #MQTT / Room variables
        self.room_code: str = ""
        self.room_status: str = "pending"  
        self.timestamp: str = ""
        
        self.num_players: int = 0
        self.card_gain: int = 0
        self.starting_card: int = 0
        
        self.players_data = [] 
        
        # Mapping: Player Order (int) -> Angle (float)
        self.player_angle_map: Dict[int, float] = {}

        # Calibration internal vars
        self.collected_angles: List[float] = []
        self.calibration_start_time = None
        # Based on rotator_node: speed 0.5 rad/s. 360 deg = 6.28 rad. Time ~12.5s.
        # We give it 18s to be safe.
        self.calibration_duration = 18.0 

        # --- Loop for state machine --- #
        self.timer = self.create_timer(0.1, self.state_machine_step)


    # ====== CALLBACKS ==============================================

    def button_pressed_callback(self, msg: Bool):
        if msg.data == True:
            self.get_logger().info("Button Pressed")
            self.is_button_pressed = True
    
    def room_config_callback(self, msg: RoomConfig):
        """Handle room configuration from MQTT and extract ALL data."""
        if self.state != DealerState.WAIT_CONFIG:
            return
        
        if msg.room_code != self.room_code:
            self.get_logger().warn(f"Ignored old/wrong room data: {msg.room_code} (Expected: {self.room_code})")
            return
        
        self.room_code = msg.room_code
        self.room_status = msg.room_status
        self.timestamp = msg.timestamp
        self.num_players = msg.num_players
        self.card_gain = msg.card_gain
        self.starting_card = msg.starting_card
        
        self.players_data = []
        for p in msg.players:
            player_info = {
                "name": p.name,
                "skipped": p.skipped,
                "order": p.order
            }
            self.players_data.append(player_info)

        self.get_logger().info(f"--- Config Received ---")
        self.get_logger().info(f"Status: {self.room_status} | Code: {self.room_code}")
        self.get_logger().info(f"Game Settings: Start={self.starting_card}, Gain={self.card_gain}")
        self.get_logger().info(f"Players ({self.num_players}): {self.players_data}")

    # [ADDED] Callback for IR Reader
    def ir_angle_callback(self, msg: Float32):
        # We only care about angles if we are currently calibrating
        if self.state == DealerState.CALIBRATING_PLAYERS:
            angle = msg.data
            self.collected_angles.append(angle)
            self.get_logger().info(f"[Calibration] Detected object at {angle:.2f} deg")
            
            # Update LCD to show progress
            found = len(self.collected_angles)
            self.publish_lcd_display(2, f"Found: {found}/{self.num_players}")

    # ====== UTILS / HELPERS ==============================================

    def map_players_to_angles(self) -> bool:
        """
        Sorts collected angles and player orders, then maps them.
        Returns True if successful, False if count mismatch.
        """
        # 1. Check counts
        if len(self.collected_angles) != self.num_players:
            self.get_logger().error(f"Mismatch! Found {len(self.collected_angles)} angles, expected {self.num_players} players.")
            return False

        # 2. Sort angles (Assuming robot rotates CCW/CW consistently, sorted angles correspond to seated order)
        # Note: Depending on your start position, there might be a wrap-around (e.g. 170, -170). 
        # For simplicity, we assume standard sorting works for non-wrap cases or simple setups.
        sorted_angles = sorted(self.collected_angles)
        
        # 3. Sort players by 'order' field
        sorted_players = sorted(self.players_data, key=lambda x: x['order'])

        # 4. Map them
        self.player_angle_map = {}
        self.get_logger().info("--- Player Mapping Result ---")
        for i in range(self.num_players):
            p_order = sorted_players[i]['order']
            p_name = sorted_players[i]['name']
            p_angle = sorted_angles[i]
            
            self.player_angle_map[p_order] = p_angle
            self.get_logger().info(f"  Player {p_order} ({p_name}) -> {p_angle:.2f} deg")
            
        return True

    def publish_state(self):
        state_msg = String()
        state_msg.data = self.state.name
        self.dealer_state_pub.publish(state_msg)

    def publish_lcd_display(self,line: int, message:str):
        message_msg = String()
        message_msg.data = message
        if line == 1:
            self.lcd_line1.publish(message_msg)
        elif line == 2:
            self.lcd_line2.publish(message_msg)

    def on_enter_state(self, new_state):
            """Handle LCD updates and logic ONLY ONCE when entering a new state."""
            self.publish_state() 
            self.get_logger().info(f"--- Entering State: {new_state.name} ---")

            if new_state == DealerState.IDLE:
                self.publish_lcd_display(1, "PRESSED TO")
                self.publish_lcd_display(2, "START")

            elif new_state == DealerState.SETUP:
                self.publish_lcd_display(1, "Creating Room...")
                self.publish_lcd_display(2, "Please Wait")

            elif new_state == DealerState.WAIT_CONFIG:
                self.publish_lcd_display(1, "Room Code:")
                self.publish_lcd_display(2, self.room_code)

            elif new_state == DealerState.CALIBRATING_PLAYERS:
                self.publish_lcd_display(1, "Calibrating...")
                self.publish_lcd_display(2, f"Found: 0/{self.num_players}")
                
                # [ADDED] Start Calibration Logic
                self.collected_angles = []
                
                # 1. Command Rotator to spin 360 degrees
                rot_msg = Float32()
                rot_msg.data = 360.0
                self.rotator_pub.publish(rot_msg)
                
                # 2. Record start time
                self.calibration_start_time = self.get_clock().now()
                self.get_logger().info("Sent 360 deg command. Scanning for players...")

    # ====== STATE MACHINE STEP ==============================================
    def state_machine_step(self):
        
        # 1. GLOBAL ENTRY LOGIC
        if self.state != self.previous_state:
            self.on_enter_state(self.state)
            self.previous_state = self.state

        # 2. STATE HANDLERS
        if self.state == DealerState.IDLE:
            if self.is_button_pressed:  
                self.is_button_pressed = False
                self.state = DealerState.SETUP
                self.get_logger().info("Button Pressed")

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

        elif self.state == DealerState.WAIT_CONFIG:
            if self.room_status == "running":
                self.get_logger().info("Room Started. Proceeding to calibration.")
                self.state = DealerState.CALIBRATING_PLAYERS
            elif self.room_status == "cancelled":
                self.state = DealerState.IDLE

        elif self.state == DealerState.CALIBRATING_PLAYERS:
            # [ADDED] Wait for the rotation to complete (Time-based)
            now = self.get_clock().now()
            elapsed_time = (now - self.calibration_start_time).nanoseconds / 1e9
            
            # Check if timeout reached (meaning rotation should be done)
            if elapsed_time > self.calibration_duration:
                self.get_logger().info("Calibration Rotation Finished.")
                
                # Try to map players
                success = self.map_players_to_angles()
                
                if success:
                    self.get_logger().info("Calibration Successful!")
                    self.state = DealerState.WAIT_START
                else:
                    self.get_logger().error("Calibration Failed (Player Count Mismatch).")
                    self.publish_lcd_display(1, "Calib Error")
                    self.publish_lcd_display(2, "Retry/Restart")
                    # For now, go to ERROR. You could add logic to retry here.
                    self.state = DealerState.ERROR

        elif self.state == DealerState.WAIT_START:
            # TODO: wait for START command (LCD and MQTT)
            pass

        elif self.state == DealerState.DEALING_ROUND:
            # TODO: implement dealing logic
            pass

        elif self.state == DealerState.WAIT_NEXT:
            # TODO: wait for NEXT/END command
            pass

        elif self.state == DealerState.GAME_END:
            pass

        elif self.state == DealerState.ERROR:
            # Simple recovery: press button to go IDLE
            if self.is_button_pressed:
                self.is_button_pressed = False
                self.state = DealerState.IDLE

# ====== MAIN ===============================================================

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