#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import RPi.GPIO as GPIO
    from RPLCD.gpio import CharLCD
    LCD_OK = True
except Exception:
    LCD_OK = False


def load_card_symbols(lcd):
    # 8 custom chars: 0..7
    chars = {
        # ♠ (spade)
        0: [0b00100,0b01110,0b11111,0b11111,0b01110,0b00100,0b00100,0b01110],
        # ♥ (heart)
        1: [0b00000,0b01010,0b11111,0b11111,0b11111,0b01110,0b00100,0b00000],
        # ♦ (diamond)
        2: [0b00100,0b01110,0b11111,0b11111,0b11111,0b01110,0b00100,0b00000],
        # ♣ (club)
        3: [0b00100,0b01110,0b00100,0b11111,0b11111,0b01110,0b00100,0b01110],
        # ♤ (outline spade)
        4: [0b00100,0b01110,0b10001,0b10001,0b01010,0b00100,0b00100,0b01110],
        # ♧ (outline club)
        5: [0b00100,0b01010,0b00100,0b10001,0b10001,0b01010,0b00100,0b01110],
        # ♡ (outline heart)
        6: [0b00000,0b01010,0b10001,0b10001,0b10001,0b01010,0b00100,0b00000],
        # ♢ (outline diamond)
        7: [0b00100,0b01010,0b10001,0b10001,0b10001,0b01010,0b00100,0b00000],
    }

    for idx, bitmap in chars.items():
        lcd.create_char(idx, bitmap)


# token -> custom char index
TOKEN_MAP = {
    "s/": 0,  # spade filled
    "h/": 1,  # heart filled
    "d/": 2,  # diamond filled
    "c/": 3,  # club filled
    "S/": 4,  # spade outline
    "C/": 5,  # club outline
    "H/": 6,  # heart outline
    "D/": 7,  # diamond outline
}


def render_lcd(text: str) -> str:
    """
    Converts tokens like 'h/' -> chr(1).
    Also handles '//' escape sequence.
    """
    if not text:
        return ""

    out = []
    i = 0
    n = len(text)

    while i < n:
        # escape: '//' -> '/'
        if i + 1 < n and text[i] == "/" and text[i + 1] == "/":
            out.append("/")
            i += 2
            continue

        # token: 2 chars, e.g. 'h/' 's/'
        if i + 1 < n:
            tok = text[i:i + 2]
            idx = TOKEN_MAP.get(tok)
            if idx is not None:
                out.append(chr(idx))
                i += 2
                continue

        out.append(text[i])
        i += 1

    return "".join(out)


class LCDNode(Node):
    def __init__(self):
        super().__init__('lcd_node')

        # GPIO mapping (BCM)
        self.pin_rs = 26
        self.pin_e = 19
        self.pins_data = [13, 6, 5, 11]  # D4 D5 D6 D7

        self.lcd = None
        self.lcd_ready = False

        # line buffers
        self.line1 = ""
        self.line2 = ""
        self.last_line1 = None
        self.last_line2 = None

        # Subscribe two topics: lcd1 -> row0, lcd2 -> row1
        self.create_subscription(String, '/lcd/display/line1', self.cb_lcd1, 10)
        self.create_subscription(String, '/lcd/display/line2', self.cb_lcd2, 10)

        if not LCD_OK:
            self.get_logger().warning("LCD library not available")
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self.lcd = CharLCD(
            numbering_mode=GPIO.BCM,
            pin_rs=self.pin_rs,
            pin_e=self.pin_e,
            pins_data=self.pins_data,
            cols=16,
            rows=2
        )
        self.lcd_ready = True

        # Load custom symbols
        load_card_symbols(self.lcd)

        # Clear and show a short ready screen
        self.lcd.clear()
        self._write_line(0, "LCD READY")
        self._write_line(1, "Auto-Center Mode")

        self.get_logger().info("LCD Node Initialize")
        self.get_logger().info("Subscribed to: /lcd/display/line1, /lcd/display/line2")

    def _write_line(self, row: int, text: str):
        if not self.lcd_ready or self.lcd is None:
            return
        
        # 1. Render symbols first (e.g. "h/" becomes a single char of length 1)
        rendered_text = render_lcd(text)
        
        # 2. Auto-center to 16 characters
        # Using .center(16) works correctly because rendered symbols count as length 1
        final_text = rendered_text.center(16)
        
        # 3. Truncate if longer than 16 (just in case)
        final_text = final_text[:16]

        self.lcd.cursor_pos = (row, 0)
        self.lcd.write_string(final_text)

    def _refresh(self):
        # update only changed lines to reduce flicker
        if self.line1 != self.last_line1:
            self._write_line(0, self.line1)
            self.last_line1 = self.line1
        if self.line2 != self.last_line2:
            self._write_line(1, self.line2)
            self.last_line2 = self.line2

    def cb_lcd1(self, msg: String):
        self.line1 = (msg.data or "").strip()
        self.get_logger().info(f"LCD1 <- {self.line1}")
        self._refresh()

    def cb_lcd2(self, msg: String):
        self.line2 = (msg.data or "").strip()
        self.get_logger().info(f"LCD2 <- {self.line2}")
        self._refresh()

    def destroy_node(self):
        if self.lcd_ready and self.lcd is not None:
            try:
                self.lcd.clear()
                self.lcd.close(clear=True)
            except Exception:
                pass
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LCDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()