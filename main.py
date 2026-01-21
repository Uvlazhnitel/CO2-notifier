# ESP32-S3 + SCD41 (I2C0 GPIO8/9) + SH1106 (I2C1 GPIO13/12)
# 3 screens: Temperature / CO2 / Humidity
# Normal mode (~5s) for faster response

from machine import Pin, I2C
import time
import sh1106
import framebuf

# ----------------- Addresses / pins -----------------
SCD4X_ADDR = 0x62
OLED_ADDR  = 0x3C

SCD_SDA  = 8
SCD_SCL  = 9
OLED_SDA = 13
OLED_SCL = 12

W, H = 128, 64

# ----------------- Measurement mode -----------------
# normal: update ~5s
# lowpower: update ~30s
MEAS_MODE = "normal"      # "normal" or "lowpower"
READY_POLL_MS = 350       # how often to poll data_ready

# ----------------- Sensor settings -----------------
ASC_ENABLED = True
ALTITUDE_M = 0

# ----------------- Screen rotation -----------------
SCREEN_DURATION_SEC = 5

# ----------------- Filters (EMA) -----------------
EMA_CO2_ALPHA = 0.35
EMA_T_ALPHA   = 0.20
EMA_RH_ALPHA  = 0.20

# ----------------- CO2 quality thresholds -----------------
CO2_GOOD_MAX = 800
CO2_OK_MAX   = 1200

# ----------------- Ventilation trigger (hysteresis) -----------------
VENT_ON_PPM   = 1200
VENT_OFF_PPM  = 900
VENT_NEED_N   = 2

# ----------------- Temperature offset -----------------
FORCE_TOFFSET_C = None     # set to a number (e.g. 3.1) or keep None
PERSIST_TOFFSET = False    # True only when you are sure (EEPROM writes)

RUN_TOFFSET_WIZARD = False
T_REFERENCE_C = 24.6
PERSIST_WIZARD = False


# ----------------- Sensirion CRC helpers -----------------
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


# ----------------- SCD41 driver -----------------
class SCD41:
    def __init__(self, i2c: I2C, addr=SCD4X_ADDR):
        self.i2c = i2c
        self.addr = addr

    def _write_cmd(self, cmd: int, args_words=None):
        self.i2c.writeto(self.addr, build_cmd(cmd, args_words))

    def _read_words(self, nwords: int):
        buf = self.i2c.readfrom(self.addr, nwords * 3)
        return parse_words_with_crc(buf)

    def stop_periodic_measurement(self):
        self._write_cmd(0x3F86)
        time.sleep_ms(500)

    def start_periodic_measurement(self):
        self._write_cmd(0x21B1)  # ~5s update
        time.sleep_ms(5)

    def start_low_power_periodic_measurement(self):
        self._write_cmd(0x21AC)  # ~30s update
        time.sleep_ms(5)

    def get_data_ready_status(self) -> bool:
        self._write_cmd(0xE4B8)
        time.sleep_ms(1)
        status = self._read_words(1)[0]
        return (status & 0x07FF) != 0

    def read_measurement(self):
        self._write_cmd(0xEC05)
        time.sleep_ms(1)
        co2_raw, t_raw, rh_raw = self._read_words(3)
        co2 = co2_raw
        temp = -45.0 + 175.0 * (t_raw / 65535.0)
        rh = 100.0 * (rh_raw / 65535.0)
        return co2, temp, rh

    def set_asc_enabled(self, enabled: bool):
        self._write_cmd(0x2416, [1 if enabled else 0])
        time.sleep_ms(1)

    def set_sensor_altitude(self, altitude_m: int):
        altitude_m = int(max(0, min(3000, altitude_m)))
        self._write_cmd(0x2427, [altitude_m])
        time.sleep_ms(1)

    def persist_settings(self):
        self._write_cmd(0x3615)
        time.sleep_ms(800)

    def get_temperature_offset(self) -> float:
        self._write_cmd(0x2318)
        time.sleep_ms(1)
        w = self._read_words(1)[0]
        return (w * 175.0) / 65535.0

    def set_temperature_offset(self, offset_c: float):
        offset_c = float(offset_c)
        if offset_c < 0:
            offset_c = 0.0
        if offset_c > 20:
            offset_c = 20.0
        word = int(round(offset_c * 65535.0 / 175.0))
        self._write_cmd(0x241D, [word])
        time.sleep_ms(1)


# ----------------- OLED: scaled text -----------------
def text_scaled(oled, s, x, y, scale=3):
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


# ----------------- Icons -----------------
def draw_temp_icon(oled, x, y):
    oled.rect(x+3, y, 4, 12, 1)
    oled.fill_rect(x+1, y+10, 8, 6, 1)
    oled.fill_rect(x+2, y+11, 6, 4, 0)
    oled.pixel(x+7, y+3, 1)
    oled.pixel(x+7, y+6, 1)

def draw_humidity_icon(oled, x, y):
    oled.pixel(x+4, y, 1)
    oled.pixel(x+3, y+1, 1)
    oled.pixel(x+5, y+1, 1)
    for i in range(2, 10):
        w = 3 + min(i-2, 4)
        oled.pixel(x+4-w//2, y+i, 1)
        oled.pixel(x+4+w//2, y+i, 1)
    oled.pixel(x+3, y+10, 1)
    oled.pixel(x+5, y+10, 1)


# ----------------- Helpers -----------------
def quality_label(ppm_int):
    if ppm_int < CO2_GOOD_MAX:
        return "GOOD"
    elif ppm_int < CO2_OK_MAX:
        return "OK"
    else:
        return "BAD"

def ema(prev, x, alpha):
    return x if prev is None else (prev + alpha * (x - prev))

def vent_banner(oled):
    oled.fill_rect(0, 56, 128, 8, 1)
    oled.text("VENTILATE!", 28, 57, 0)


# ----------------- Screens -----------------
def render_co2_screen(oled, co2v, vent_flag):
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
    text_scaled(oled, co2_str, x_num, y_num, scale=scale_num)

    oled.text("ppm", 52, 44)

    scale_lbl = 2
    x_lbl = centered_x_for_scaled(label, scale_lbl)
    text_scaled(oled, label, x_lbl, 46, scale=scale_lbl)

    oled.show()

def render_temp_screen(oled, tv, vent_flag):
    oled.fill(0)
    draw_temp_icon(oled, 4, 2)
    oled.text("TEMPERATURE", 20, 2)

    t = float(tv)
    sign = "-" if t < 0 else ""
    t = abs(t)

    t_int = int(t)
    t_dec = int(round((t - t_int) * 10))
    if t_dec == 10:
        t_int += 1
        t_dec = 0

    big = sign + str(t_int)
    dec = ".%d" % t_dec

    if len(big) <= 2:
        scale_big = 4
    elif len(big) == 3:
        scale_big = 3
    else:
        scale_big = 2

    scale_dec = 2

    big_w = len(big) * 8 * scale_big
    dec_w = len(dec) * 8 * scale_dec
    total_w = big_w + dec_w + 2
    x0 = (W - total_w) // 2

    y_big = 20
    y_dec = y_big + (scale_big*8 - scale_dec*8)

    text_scaled(oled, big, x0, y_big, scale=scale_big)
    text_scaled(oled, dec, x0 + big_w + 2, y_dec, scale=scale_dec)

    oled.text("C", 120, 35)

    if vent_flag:
        vent_banner(oled)

    oled.show()

def render_hum_screen(oled, rhv, vent_flag):
    oled.fill(0)
    draw_humidity_icon(oled, 4, 2)
    oled.text("HUMIDITY", 22, 2)

    rh_str = "{:.0f}".format(rhv)
    text_scaled(oled, rh_str, 28, 20, scale=5)
    oled.text("%", 112, 35)

    if vent_flag:
        vent_banner(oled)

    oled.show()

def render_screen(oled, screen, co2v, tv, rhv, vent_flag):
    if screen == 0:
        render_temp_screen(oled, tv, vent_flag)
    elif screen == 1:
        render_co2_screen(oled, co2v, vent_flag)
    else:
        render_hum_screen(oled, rhv, vent_flag)


# ----------------- Main -----------------
def main():
    i2c_scd  = I2C(0, sda=Pin(SCD_SDA),  scl=Pin(SCD_SCL),  freq=100_000)
    i2c_oled = I2C(1, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=400_000)

    scd_scan = i2c_scd.scan()
    oled_scan = i2c_oled.scan()
    print("I2C0 scan (SCD): ", [hex(a) for a in scd_scan])
    print("I2C1 scan (OLED):", [hex(a) for a in oled_scan])

    oled = sh1106.SH1106_I2C(W, H, i2c_oled, addr=OLED_ADDR)
    oled.sleep(False)
    oled.fill(0)
    oled.text("Starting...", 30, 28)
    oled.show()

    if SCD4X_ADDR not in scd_scan:
        oled.fill(0)
        oled.text("SCD41 not found", 10, 24)
        oled.show()
        raise RuntimeError("SCD41 not found at 0x62")

    scd = SCD41(i2c_scd)

    try:
        scd.stop_periodic_measurement()
    except:
        pass

    scd.set_asc_enabled(ASC_ENABLED)
    scd.set_sensor_altitude(ALTITUDE_M)

    toffset = None
    try:
        toffset = scd.get_temperature_offset()
        print("Current Toffset:", toffset)
    except Exception as e:
        print("Toffset read error:", e)

    if FORCE_TOFFSET_C is not None:
        scd.set_temperature_offset(FORCE_TOFFSET_C)
        time.sleep_ms(10)
        try:
            toffset = scd.get_temperature_offset()
        except:
            toffset = FORCE_TOFFSET_C
        print("New Toffset (not persisted):", toffset)
        if PERSIST_TOFFSET:
            scd.persist_settings()
            print("Toffset persisted.")

    if RUN_TOFFSET_WIZARD:
        scd.start_periodic_measurement()
        print("Wizard: waiting fresh sample...")
        while not scd.get_data_ready_status():
            time.sleep_ms(READY_POLL_MS)
        _, t_scd, _ = scd.read_measurement()

        prev = toffset if toffset is not None else 4.0
        new_off = (t_scd - T_REFERENCE_C) + prev
        if new_off < 0:
            new_off = 0.0
        if new_off > 20:
            new_off = 20.0

        scd.stop_periodic_measurement()
        scd.set_temperature_offset(new_off)
        time.sleep_ms(10)
        toffset = scd.get_temperature_offset()
        print("Wizard set Toffset:", toffset)
        if PERSIST_WIZARD:
            scd.persist_settings()
            print("Wizard Toffset persisted.")

    if MEAS_MODE == "lowpower":
        scd.start_low_power_periodic_measurement()
        print("Mode: LOW POWER (~30s)")
    else:
        scd.start_periodic_measurement()
        print("Mode: NORMAL (~5s)")

    screen = 0
    last_switch = time.ticks_ms()
    last_ready_poll = time.ticks_ms()

    co2_f = None
    t_f   = None
    rh_f  = None

    vent_needed = False
    vent_counter = 0
    have_sample = False

    render_screen(oled, screen, 400, 0.0, 0.0, False)

    while True:
        now = time.ticks_ms()
        redraw = False

        if time.ticks_diff(now, last_switch) >= SCREEN_DURATION_SEC * 1000:
            screen = (screen + 1) % 3
            last_switch = now
            redraw = True

        if time.ticks_diff(now, last_ready_poll) >= READY_POLL_MS:
            last_ready_poll = now

            if scd.get_data_ready_status():
                co2, temp, rh = scd.read_measurement()
                print("CO2:%d ppm  T:%.2f C  RH:%.2f%%" % (co2, temp, rh))

                co2_f = ema(co2_f, co2, EMA_CO2_ALPHA)
                t_f   = ema(t_f,   temp, EMA_T_ALPHA)
                rh_f  = ema(rh_f,  rh,  EMA_RH_ALPHA)

                co2_logic = co2_f if co2_f is not None else co2

                if co2_logic >= VENT_ON_PPM:
                    vent_counter += 1
                    if vent_counter >= VENT_NEED_N:
                        vent_needed = True
                elif co2_logic <= VENT_OFF_PPM:
                    vent_needed = False
                    vent_counter = 0
                else:
                    vent_counter = 0

                have_sample = True
                redraw = True

        if redraw and have_sample:
            render_screen(
                oled,
                screen,
                co2_f if co2_f is not None else co2,
                t_f   if t_f   is not None else temp,
                rh_f  if rh_f  is not None else rh,
                vent_needed
            )

        time.sleep_ms(20)

main()
