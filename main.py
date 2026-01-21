# ESP32-S3 + SCD41 (I2C0 GPIO8/9) + SH1106 (I2C1 GPIO13/12)

from machine import Pin, I2C
import time
import sh1106
import framebuf

SCD4X_ADDR = 0x62
OLED_ADDR  = 0x3C

SCD_SDA  = 8
SCD_SCL  = 9
OLED_SDA = 13
OLED_SCL = 12

W, H = 128, 64

USE_LOW_POWER = True
ASC_ENABLED = True
ALTITUDE_M = 0

SCREEN_DURATION_SEC = 5

EMA_CO2_ALPHA = 0.25
EMA_T_ALPHA   = 0.15
EMA_RH_ALPHA  = 0.15

CO2_GOOD_MAX = 800
CO2_OK_MAX   = 1200

VENT_ON_PPM   = 1200
VENT_OFF_PPM  = 900
VENT_NEED_N   = 2
# --------------------------------------------

# ====== I2C ======
i2c_scd  = I2C(0, sda=Pin(SCD_SDA),  scl=Pin(SCD_SCL),  freq=100_000)
i2c_oled = I2C(1, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=400_000)

print("I2C0 scan (SCD): ", [hex(a) for a in i2c_scd.scan()])
print("I2C1 scan (OLED):", [hex(a) for a in i2c_oled.scan()])

# ====== OLED ======
oled = sh1106.SH1106_I2C(W, H, i2c_oled, addr=OLED_ADDR)
oled.sleep(False)
oled.fill(0)
oled.text("Starting...", 30, 28)
oled.show()

# ====== Big text ======
def text_scaled(s, x, y, scale=3):
    bw = len(s) * 8
    bh = 8
    buf = bytearray((bw * bh + 7) // 8)
    fb = framebuf.FrameBuffer(buf, bw, bh, framebuf.MONO_HLSB)
    fb.fill(0)
    fb.text(s, 0, 0, 1)

    for yy in range(bh):
        for xx in range(bw):
            if fb.pixel(xx, yy):
                base_x = x + xx * scale
                base_y = y + yy * scale
                if base_x >= W or base_y >= H:
                    continue
                for dy in range(scale):
                    py = base_y + dy
                    if py < 0 or py >= H:
                        continue
                    for dx in range(scale):
                        px = base_x + dx
                        if 0 <= px < W:
                            oled.pixel(px, py, 1)

def centered_x_for_scaled(text, scale):
    px = len(text) * 8 * scale
    x = (W - px) // 2
    return 0 if x < 0 else x

def draw_temp_icon(x, y):
    oled.rect(x+3, y, 4, 12, 1)
    oled.fill_rect(x+1, y+10, 8, 6, 1)
    oled.fill_rect(x+2, y+11, 6, 4, 0)
    oled.pixel(x+7, y+3, 1)
    oled.pixel(x+7, y+6, 1)

def draw_humidity_icon(x, y):
    oled.pixel(x+4, y, 1)
    oled.pixel(x+3, y+1, 1)
    oled.pixel(x+5, y+1, 1)
    for i in range(2, 10):
        w = 3 + min(i-2, 4)
        oled.pixel(x+4-w//2, y+i, 1)
        oled.pixel(x+4+w//2, y+i, 1)
    oled.pixel(x+3, y+10, 1)
    oled.pixel(x+5, y+10, 1)

# ====== CRC / SCD41 ======
def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x31
            else:
                crc = (crc << 1) & 0xFF
    return crc

def build_cmd(cmd: int, args_words=None) -> bytes:
    out = bytes([(cmd >> 8) & 0xFF, cmd & 0xFF])
    if args_words:
        for w in args_words:
            d = bytes([(w >> 8) & 0xFF, w & 0xFF])
            out += d + bytes([crc8(d)])
    return out

def parse_words_with_crc(buf: bytes):
    if len(buf) % 3 != 0:
        raise ValueError("Bad length")
    words = []
    for i in range(0, len(buf), 3):
        d = buf[i:i+2]
        c = buf[i+2]
        if crc8(d) != c:
            raise ValueError("CRC mismatch")
        words.append((d[0] << 8) | d[1])
    return words

class SCD41:
    def __init__(self, i2c: I2C, addr=SCD4X_ADDR):
        self.i2c = i2c
        self.addr = addr

    def _write_cmd(self, cmd: int, args_words=None):
        self.i2c.writeto(self.addr, build_cmd(cmd, args_words))

    def _read(self, nbytes: int) -> bytes:
        return self.i2c.readfrom(self.addr, nbytes)

    def stop_periodic_measurement(self):
        self._write_cmd(0x3F86)
        time.sleep_ms(500)

    def start_periodic_measurement(self):
        self._write_cmd(0x21B1)
        time.sleep_ms(5)

    def start_low_power_periodic_measurement(self):
        self._write_cmd(0x21AC)
        time.sleep_ms(5)

    def get_data_ready_status(self) -> bool:
        self._write_cmd(0xE4B8)
        time.sleep_ms(1)
        status = parse_words_with_crc(self._read(3))[0]
        return (status & 0x07FF) != 0

    def read_measurement(self):
        self._write_cmd(0xEC05)
        time.sleep_ms(1)
        co2_raw, t_raw, rh_raw = parse_words_with_crc(self._read(9))
        co2 = co2_raw
        temp = -45.0 + 175.0 * (t_raw / 65535.0)
        rh = 100.0 * (rh_raw / 65535.0)
        return co2, temp, rh

    def set_asc_enabled(self, enabled: bool):
        self._write_cmd(0x2416, [1 if enabled else 0])
        time.sleep_ms(1)

    def set_sensor_altitude(self, altitude_m: int):
        altitude_m = int(max(0, min(65535, altitude_m)))
        self._write_cmd(0x2427, [altitude_m])
        time.sleep_ms(1)

def ema(prev, x, alpha):
    return x if prev is None else (prev + alpha * (x - prev))

def quality_label(ppm_int):
    if ppm_int < CO2_GOOD_MAX:
        return "GOOD"
    elif ppm_int < CO2_OK_MAX:
        return "OK"
    else:
        return "BAD"

# ====== init sensor ======
if SCD4X_ADDR not in i2c_scd.scan():
    oled.fill(0)
    oled.text("SCD41 not found", 10, 24)
    oled.show()
    raise RuntimeError("SCD41 not found on I2C0")

scd = SCD41(i2c_scd)
try:
    scd.stop_periodic_measurement()
except Exception:
    pass

scd.set_asc_enabled(ASC_ENABLED)
scd.set_sensor_altitude(ALTITUDE_M)

if USE_LOW_POWER:
    scd.start_low_power_periodic_measurement()
else:
    scd.start_periodic_measurement()

# ====== state ======
screen = 0
last_switch = time.ticks_ms()

co2, temp, rh = 400, 25.0, 50.0
co2_f = None
t_f   = None
rh_f  = None

vent_needed = False
vent_counter = 0

def vent_banner():
    oled.fill_rect(0, 56, 128, 8, 1)
    oled.text("VENTILATE!", 28, 57, 0)

def render_co2_screen(co2v, vent_flag):
    oled.fill(0)

    co2_disp = int(round(float(co2v)))
    label = quality_label(co2_disp)

    oled.text("AIR QUALITY", 24, 0)
    if vent_flag:
        oled.text("!", 120, 0)

    co2_str = str(co2_disp)
    if len(co2_str) <= 3:
        scale_num = 4      
        y_num = 10
    else:
        scale_num = 3
        y_num = 12
    x_num = centered_x_for_scaled(co2_str, scale_num)
    text_scaled(co2_str, x_num, y_num, scale=scale_num)

    oled.text("ppm", 52, 44)

    scale_lbl = 2
    x_lbl = centered_x_for_scaled(label, scale_lbl)
    text_scaled(label, x_lbl, 46, scale=scale_lbl)

    oled.show()
# --------------------------------------------------------

def render_screen(screen, co2v, tv, rhv, vent_flag):
    if screen == 0:
        oled.fill(0)
        draw_temp_icon(4, 2)
        oled.text("TEMPERATURE", 20, 2)
        temp_str = "{:.0f}".format(tv)
        text_scaled(temp_str, 25, 20, scale=5)
        oled.text("C", 112, 35)
        if vent_flag:
            vent_banner()
        oled.show()

    elif screen == 1:
        render_co2_screen(co2v, vent_flag)

    else:
        oled.fill(0)
        draw_humidity_icon(4, 2)
        oled.text("HUMIDITY", 22, 2)
        rh_str = "{:.0f}".format(rhv)
        text_scaled(rh_str, 28, 20, scale=5)
        oled.text("%", 112, 35)
        if vent_flag:
            vent_banner()
        oled.show()

# ====== loop ======
last_oled_update = time.ticks_ms()
OLED_UPDATE_MS = 300

while True:
    try:
        if scd.get_data_ready_status():
            co2, temp, rh = scd.read_measurement()
            print("CO2:%d ppm  T:%.2f C  RH:%.2f%%" % (co2, temp, rh))

            co2_f = ema(co2_f, co2, EMA_CO2_ALPHA)
            t_f   = ema(t_f,   temp, EMA_T_ALPHA)
            rh_f  = ema(rh_f,  rh,  EMA_RH_ALPHA)

            if co2 >= VENT_ON_PPM:
                vent_counter += 1
                if vent_counter >= VENT_NEED_N:
                    vent_needed = True
            elif co2 <= VENT_OFF_PPM:
                vent_needed = False
                vent_counter = 0

        if time.ticks_diff(time.ticks_ms(), last_switch) >= SCREEN_DURATION_SEC * 1000:
            screen = (screen + 1) % 3
            last_switch = time.ticks_ms()
            oled.fill(0)
            oled.show()
            time.sleep_ms(30)

        if time.ticks_diff(time.ticks_ms(), last_oled_update) >= OLED_UPDATE_MS:
            last_oled_update = time.ticks_ms()
            render_screen(
                screen,
                co2_f if co2_f is not None else co2,
                t_f   if t_f   is not None else temp,
                rh_f  if rh_f  is not None else rh,
                vent_needed
            )

        time.sleep_ms(50)

    except Exception as e:
        print("Error:", e)
        oled.fill(0)
        oled.text("Error:", 0, 0)
        oled.text(str(e)[:16], 0, 16)
        oled.show()
        time.sleep(2)
