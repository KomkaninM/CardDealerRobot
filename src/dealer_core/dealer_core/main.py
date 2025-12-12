#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Float32, Int32, Bool

from enum import Enum, auto
from typing import Dict

from RoomCodeGenerator import *


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
        # Put publusher topic here
        self.dealer_state_pub = self.create_publisher(String, "/dealer/state", qos)
        self.dealer_event_pub = self.create_publisher(String, "/dealer/event", qos)


        # --- Subscribers --- #
        # Put subscriber topic here
        self.mqtt_room_config_sub = self.create_subscription(
            String, "/mqtt/room_config", self.room_config_callback, qos
        )
        self.mqtt_control_cmd_sub = self.create_subscription(
            String, "/mqtt/control_cmd", self.control_cmd_callback, qos
        )

        # ---Variable Declaration --- #
        self.state = DealerState.IDLE
        self.previous_state = DealerState.IDLE

        #MQTT / Room variables
        self.room_code: str = ""
        self.config: Dict = {}
        self.player_angle_map: Dict[int, float] = {}
        self.current_round: int = 0
        self.num_players: int = 0
        self.max_rounds: int = 0
        self.player_cards: Dict[int, str] = {}
        self.pending_command: str = "NONE"

        # --- Loop for state machine --- #
        self.timer = self.create_timer(0.1, self.state_machine_step)


    # ====== CALLBACKS (SUBSCRIBERS) ==============================================

    def room_config_callback(self, msg: String):
        """Handle room configuration from MQTT."""
        # TODO: parse JSON, store config, change state
        pass

    def control_cmd_callback(self, msg: String):
        """Handle control commands (START, NEXT, END, etc.) from MQTT."""
        # TODO: parse JSON, set pending_command
        pass

    
    # ====== UTILS / HELPERS (STRUCTURE ONLY) ================================

    def reset_variables(self):
        """Reset all internal variables and go to IDLE state."""
        # TODO: implement
        pass

    def publish_state(self):
        """Publish current state to /dealer/state."""
        state_msg = String()
        state_msg.data = self.state.name
        self.dealer_state_pub.publish(state_msg)
        # self.get_logger().info(f"State changed to: {self.state.name}")

    def publish_event(self, event: str):
        """Publish an event to /dealer/event."""
        event_msg = String()
        event_msg.data = event
        self.dealer_event_pub.publish(event_msg)
        # self.get_logger().info(f"Event published: {event}")



    # ====== STATE MACHINE STEP ==============================================
    # --- Main logic loop for state machine --- #
    def state_machine_step(self):
        if self.state == DealerState.IDLE:
            self.publish_state()
            # TODO: handle IDLE logic
            pass

        elif self.state == DealerState.SETUP:
            self.publish_state()
            # TODO: handle SETUP logic (e.g. generate room code)
            new_room = create_unique_room()
            print("New minimal room in MongoDB:")
            print(new_room)

        elif self.state == DealerState.WAIT_CONFIG:
            # TODO: wait for config from MQTT
            pass

        elif self.state == DealerState.CALIBRATING_PLAYERS:
            # TODO: manage calibration flow with IR + Rotater
            pass

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
            # TODO: finalize game and go back to IDLE
            pass

        elif self.state == DealerState.ERROR:
            # TODO: error handling & recovery
            pass

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