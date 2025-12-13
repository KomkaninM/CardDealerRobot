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


def fit16(text: str) -> str:
    return (text[:16]).ljust(16)


# =========================
# Arduino bitmaps (ของคุณ)
# =========================
name1x10 = [0b11111,0b11110,0b10000,0b10000,0b10000,0b10000,0b11111,0b11111]

name0x0  = [0b00110,0b01111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111]
name0x1  = [0b01100,0b11110,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111]

name0x3  = [0b00001,0b00011,0b00011,0b00111,0b00111,0b01111,0b01111,0b11111]
name0x4  = [0b10000,0b11000,0b11000,0b11100,0b11100,0b11110,0b11110,0b11111]

name0x6  = [0b00001,0b00011,0b00111,0b01111,0b01111,0b11111,0b11111,0b11111]
name0x7  = [0b10000,0b11000,0b11100,0b11110,0b11110,0b11111,0b11111,0b11111]

name0x9  = [0b00001,0b00011,0b00111,0b00111,0b00011,0b00001,0b01110,0b11111]
name0x10 = [0b10000,0b11000,0b11100,0b11100,0b11000,0b10000,0b01110,0b11111]

name1x0  = [0b01111,0b01111,0b00111,0b00111,0b00011,0b00011,0b00001,0b00001]
name1x1  = [0b11110,0b11110,0b11100,0b11100,0b11000,0b11000,0b10000,0b10000]

name1x3  = [0b11111,0b01111,0b01111,0b00111,0b00111,0b00011,0b00011,0b00001]
name1x4  = [0b11111,0b11110,0b11110,0b11100,0b11100,0b11000,0b11000,0b10000]

name1x6  = [0b11111,0b01111,0b00111,0b00011,0b00001,0b00001,0b11111,0b11111]
name1x7  = [0b11111,0b11110,0b11100,0b11000,0b10000,0b10000,0b11111,0b11111]

name1x9  = [0b11111,0b01111,0b00001,0b00001,0b00001,0b00001,0b11111,0b11111]


# =========================
# สร้าง 2x2 tiles ต่อ 1 รูปใหญ่
#   tiles = [TL, TR, BL, BR]
# =========================
HEART_TILES   = [name0x0,  name0x1,  name1x0,  name1x1]
DIAMOND_TILES = [name0x3,  name0x4,  name1x3,  name1x4]
SPADE_TILES   = [name0x6,  name0x7,  name1x6,  name1x7]
CLUB_TILES    = [name0x9,  name0x10, name1x9,  name1x10]


def draw_two_big_cards(lcd, left_tiles, right_tiles, left_col=0, right_col=4):
    """
    โหลด custom char 0..7 สำหรับ 2 ใบ (2x2 + 2x2 = 8 tiles) แล้ววาดเต็ม 2 แถว
    left_col/right_col คือคอลัมน์เริ่มของใบซ้าย/ขวา
    """
    # โหลดเข้า CGRAM 0..7
    tiles = left_tiles + right_tiles  # 8 tiles
    for i, tile in enumerate(tiles):
        lcd.create_char(i, tile)

    # ใบซ้าย (0,1 / 2,3)
    lcd.cursor_pos = (0, left_col)
    lcd.write_string(chr(0) + chr(1))
    lcd.cursor_pos = (1, left_col)
    lcd.write_string(chr(2) + chr(3))

    # ใบขวา (4,5 / 6,7)
    lcd.cursor_pos = (0, right_col)
    lcd.write_string(chr(4) + chr(5))
    lcd.cursor_pos = (1, right_col)
    lcd.write_string(chr(6) + chr(7))


class LCDNode(Node):
    def __init__(self):
        super().__init__('lcd_node')

        # GPIO mapping (BCM)
        self.pin_rs = 26
        self.pin_e  = 19
        self.pins_data = [13, 6, 5, 11]  # D4 D5 D6 D7

        self.lcd = None
        self.gpio_inited = False

        # page 0 = ♥♦, page 1 = ♠♣
        self.page = 0
        self.flip_sec = 1.5

        # ถ้าจะใช้ข้อความจาก topic ด้วย: ส่ง MODE:TEXT / MODE:CARDS
        self.page_mode = True
        self.last_msg = ""

        self.create_subscription(String, '/lcd/display', self.cb_display, 10)

        if not LCD_OK:
            self.get_logger().warning("LCD library not available")
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.gpio_inited = True

        self.lcd = CharLCD(
            numbering_mode=GPIO.BCM,
            pin_rs=self.pin_rs,
            pin_e=self.pin_e,
            pins_data=self.pins_data,
            cols=16,
            rows=2
        )

        self.get_logger().info("LCD node started")

        # เริ่มหน้า ♥♦
        self.draw_page(0)

        # timer สลับหน้า
        self.flip_timer = self.create_timer(self.flip_sec, self.flip_page)

    def draw_page(self, page: int):
        if not self.lcd:
            return
        self.lcd.clear()

        # ปรับตำแหน่งให้ “เต็มจอ” ได้ที่นี่
        # ถ้าอยากให้ไพ่ชิดซ้าย/ขวากว่านี้ เปลี่ยน right_col เป็น 5 หรือ 6
        left_col = 0
        right_col = 4

        if page == 0:
            # ♥ ♦
            draw_two_big_cards(self.lcd, HEART_TILES, DIAMOND_TILES, left_col, right_col)
        else:
            # ♠ ♣
            draw_two_big_cards(self.lcd, SPADE_TILES, CLUB_TILES, left_col, right_col)

    def flip_page(self):
        if not self.lcd or not self.page_mode:
            return
        self.page = 1 - self.page
        try:
            self.draw_page(self.page)
        except Exception as e:
            self.get_logger().warning(f"Flip page failed: {e}")

    def cb_display(self, msg: String):
        if not self.lcd:
            return

        text = (msg.data or "").strip()
        if text == self.last_msg:
            return
        self.last_msg = text

        # คุมโหมด
        if text.upper() == "MODE:CARDS":
            self.page_mode = True
            self.page = 0
            self.draw_page(0)
            self.get_logger().info("Switched to CARDS mode")
            return

        if text.upper() == "MODE:TEXT":
            self.page_mode = False
            self.lcd.clear()
            self.get_logger().info("Switched to TEXT mode")
            return

        # ถ้าอยู่โหมดไพ่ใหญ่ ไม่ให้ข้อความมาทับ
        if self.page_mode:
            return

        # โหมดข้อความ
        if "|" in text:
            line1, line2 = text.split("|", 1)
        else:
            line1, line2 = text, ""

        line1 = fit16(line1)
        line2 = fit16(line2)

        self.lcd.clear()
        self.lcd.cursor_pos = (0, 0)
        self.lcd.write_string(line1)
        self.lcd.cursor_pos = (1, 0)
        self.lcd.write_string(line2)

    def destroy_node(self):
        try:
            if self.lcd:
                self.lcd.clear()
                self.lcd.close(clear=True)
        except Exception:
            pass

        try:
            if self.gpio_inited:
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
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
