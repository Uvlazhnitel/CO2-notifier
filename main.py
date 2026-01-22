# ESP32-S3 + SCD41 (I2C0 GPIO8/9) + SH1106 (I2C1 GPIO13/12)
# 3 screens: Temperature / CO2 / Humidity
# + Telegram notify when "vent_needed" turns ON



from machine import Pin, I2C
import time
import network
import framebuf
import sh1106

try:
    import urequests as requests
except ImportError:
    raise ImportError("urequests not found. Upload urequests.py to the board.")

# ---------------- USER SETTINGS ----------------
WIFI_SSID = ""
WIFI_PASS = ""

TG_TOKEN  = ""  
TG_CHAT_ID = ""   

SEND_BOOT_MSG = True

# Telegram pacing
TG_MIN_GAP_MS = 1500            # minimal gap between any sends (safety)
TG_REMIND_MS  = 20 * 60 * 1000   # remind while still >= WARN (set small for testing, e.g. 15000)

# Wi-Fi watchdog
WIFI_RECONNECT_MS = 10_000

# ---------------- PINS / ADDRESSES ----------------
SCD4X_ADDR = 0x62
OLED_ADDR  = 0x3C

SCD_SDA  = 8
SCD_SCL  = 9
OLED_SDA = 13
OLED_SCL = 12

W, H = 128, 64

# ---------------- SENSOR MODE ----------------
MEAS_MODE      = "normal"   # "normal" (~5s) or "lowpower" (~30s)
READY_POLL_MS  = 350

ASC_ENABLED = True
ALTITUDE_M = 0

# ---------------- SCREEN ROTATION ----------------
SCREEN_DURATION_SEC = 5

# ---------------- FILTERS ----------------
EMA_CO2_ALPHA = 0.35
EMA_T_ALPHA   = 0.20
EMA_RH_ALPHA  = 0.20

# ---------------- CO2 LEVELS (HYSTERESIS) ----------------
# Level 0: OK
# Level 1: WARN (productivity threshold)
# Level 2: HIGH
# Level 3: RED (unacceptable)
WARN_ON  = 1000
WARN_OFF = 850

HIGH_ON  = 1400
HIGH_OFF = 1200

RED_ON   = 2000
RED_OFF  = 1800

# Optional temperature offset
FORCE_TOFFSET_C = None
PERSIST_TOFFSET = False


# ---------------- SMALL HELPERS ----------------
def ema(prev, x, alpha):
    return x if prev is None else (prev + alpha * (x - prev))

def level_name(lvl: int) -> str:
    return ["OK", "WARN", "HIGH", "RED"][lvl]

def url_escape(s: str) -> str:
    out = []
    for b in s.encode("utf-8"):
        if (48 <= b <= 57) or (65 <= b <= 90) or (97 <= b <= 122) or b in b"-_.~":
            out.append(chr(b))
        elif b == 32:
            out.append("%20")
        else:
            out.append("%{:02X}".format(b))
    return "".join(out)


# ---------------- WIFI ----------------
def wifi_connect(timeout_ms=15000):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        return wlan

    wlan.connect(WIFI_SSID, WIFI_PASS)
    t0 = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            break
        time.sleep_ms(200)
    return wlan


# ---------------- OPTIONAL: NTP TIME (helps some HTTPS/TLS setups) ----------------
def sync_time_ntp():
    try:
        import ntptime
        ntptime.settime()  # sets UTC
        print("NTP time synced")
    except Exception as e:
        print("NTP sync skipped/error:", e)


# ---------------- TELEGRAM ----------------
_last_tg_send = 0

def tg_send(text: str) -> bool:
    global _last_tg_send

    if not TG_TOKEN or TG_TOKEN == "YOUR_BOT_TOKEN":
        print("TG: token not set")
        return False

    # simple anti-spam gap
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_tg_send) < TG_MIN_GAP_MS:
        return False

    # GET is the most compatible in MicroPython
    url = "https://api.telegram.org/bot{}/sendMessage?chat_id={}&text={}".format(
        TG_TOKEN, TG_CHAT_ID, url_escape(text)
    )

    try:
        r = requests.get(url)
        code = getattr(r, "status_code", None)
        body = getattr(r, "text", "")
        r.close()

        print("TG HTTP:", code)
        if code is not None and code != 200:
            print("TG body:", body[:200])
            return False

        _last_tg_send = now
        return True

    except Exception as e:
        print("TG send exception:", e)
        return False


# ---------------- SENSIRION CRC ----------------
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


# ---------------- SCD41 DRIVER ----------------
class SCD41:
    def __init__(self, i2c, addr=SCD4X_ADDR):
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
        self._write_cmd(0x21B1)  # ~5s
        time.sleep_ms(5)

    def start_low_power_periodic_measurement(self):
        self._write_cmd(0x21AC)  # ~30s
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

    def set_temperature_offset(self, offset_c: float):
        offset_c = float(offset_c)
        if offset_c < 0:
            offset_c = 0.0
        if offset_c > 20:
            offset_c = 20.0
        word = int(round(offset_c * 65535.0 / 175.0))
        self._write_cmd(0x241D, [word])
        time.sleep_ms(1)


# ---------------- OLED DRAW ----------------
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
                    if 0 <= py < H:
                        for dx in range(scale):
                            px = base_x + dx
                            if 0 <= px < W:
                                oled.pixel(px, py, 1)

def centered_x_for_scaled(text, scale):
    px = len(text) * 8 * scale
    x = (W - px) // 2
    return 0 if x < 0 else x

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

def banner(oled, lvl):
    oled.fill_rect(0, 56, 128, 8, 1)
    oled.text(level_name(lvl), 54, 57, 0)

def render_co2(oled, co2v, lvl):
    oled.fill(0)
    co2i = int(round(co2v))
    lbl = level_name(lvl)

    oled.text("CO2", 56, 0)
    if lvl >= 1:
        oled.text("!", 120, 0)

    s = str(co2i)
    sc = 4 if len(s) <= 3 else 3
    y  = 10 if len(s) <= 3 else 12
    x  = centered_x_for_scaled(s, sc)
    text_scaled(oled, s, x, y, sc)

    oled.text("ppm", 52, 44)

    sc2 = 2
    x2 = centered_x_for_scaled(lbl, sc2)
    text_scaled(oled, lbl, x2, 46, sc2)

    if lvl >= 1:
        banner(oled, lvl)
    oled.show()

def render_temp(oled, tv, lvl):
    oled.fill(0)
    draw_temp_icon(oled, 4, 2)
    oled.text("TEMP", 56, 2)

    t = float(tv)
    sign = "-" if t < 0 else ""
    t = abs(t)
    ti = int(t)
    td = int(round((t - ti) * 10))
    if td == 10:
        ti += 1
        td = 0

    big = sign + str(ti)
    dec = ".%d" % td

    sb = 4 if len(big) <= 2 else (3 if len(big) == 3 else 2)
    sd = 2

    bw = len(big) * 8 * sb
    dw = len(dec) * 8 * sd
    x0 = (W - (bw + dw + 2)) // 2
    yb = 20
    yd = yb + (sb*8 - sd*8)

    text_scaled(oled, big, x0, yb, sb)
    text_scaled(oled, dec, x0 + bw + 2, yd, sd)

    oled.text("C", 120, 35)
    if lvl >= 1:
        banner(oled, lvl)
    oled.show()

def render_hum(oled, rh, lvl):
    oled.fill(0)
    draw_humidity_icon(oled, 4, 2)
    oled.text("HUM", 58, 2)

    s = "{:.0f}".format(rh)
    text_scaled(oled, s, 28, 20, 5)
    oled.text("%", 112, 35)

    if lvl >= 1:
        banner(oled, lvl)
    oled.show()

def render_screen(oled, screen, co2v, tv, rhv, lvl):
    if screen == 0:
        render_temp(oled, tv, lvl)
    elif screen == 1:
        render_co2(oled, co2v, lvl)
    else:
        render_hum(oled, rhv, lvl)


# ---------------- CO2 LEVEL FSM ----------------
def update_level(prev, co2):
    # OK -> WARN -> HIGH -> RED with hysteresis
    if prev == 0:
        if co2 >= RED_ON:  return 3
        if co2 >= HIGH_ON: return 2
        if co2 >= WARN_ON: return 1
        return 0

    if prev == 1:
        if co2 >= RED_ON:  return 3
        if co2 >= HIGH_ON: return 2
        if co2 <= WARN_OFF: return 0
        return 1

    if prev == 2:
        if co2 >= RED_ON:  return 3
        if co2 <= HIGH_OFF:
            return 1 if co2 >= WARN_ON else 0
        return 2

    # prev == 3
    if co2 <= RED_OFF:
        if co2 >= HIGH_ON: return 2
        if co2 >= WARN_ON: return 1
        return 0
    return 3


# ---------------- MAIN ----------------
def main():
    wlan = wifi_connect()
    print("Wi-Fi connected:", wlan.isconnected(), wlan.ifconfig() if wlan.isconnected() else None)

    # optional NTP (doesn't hurt)
    if wlan.isconnected():
        sync_time_ntp()

    i2c_scd  = I2C(0, sda=Pin(SCD_SDA),  scl=Pin(SCD_SCL),  freq=100_000)
    i2c_oled = I2C(1, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=400_000)

    oled = sh1106.SH1106_I2C(W, H, i2c_oled, addr=OLED_ADDR)
    oled.sleep(False)
    oled.fill(0)
    oled.text("Starting...", 30, 28)
    oled.show()

    if SCD4X_ADDR not in i2c_scd.scan():
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

    if FORCE_TOFFSET_C is not None:
        scd.set_temperature_offset(FORCE_TOFFSET_C)
        time.sleep_ms(10)
        if PERSIST_TOFFSET:
            scd.persist_settings()

    if MEAS_MODE == "lowpower":
        scd.start_low_power_periodic_measurement()
    else:
        scd.start_periodic_measurement()

    # Telegram quick self-test
    if SEND_BOOT_MSG and wlan.isconnected():
        ok = tg_send("CO2 monitor booted ✅")
        print("TG boot send:", ok)

    screen = 0
    last_switch = time.ticks_ms()
    last_ready  = time.ticks_ms()
    last_wifi   = time.ticks_ms()

    co2_f = None
    t_f   = None
    rh_f  = None

    lvl = 0
    last_remind = time.ticks_add(time.ticks_ms(), -TG_REMIND_MS)

    have_sample = False
    render_screen(oled, screen, 400.0, 0.0, 0.0, lvl)

    while True:
        now = time.ticks_ms()

        # Wi-Fi watchdog
        if time.ticks_diff(now, last_wifi) > WIFI_RECONNECT_MS:
            last_wifi = now
            if not wlan.isconnected():
                print("Wi-Fi reconnect...")
                wlan = wifi_connect()

        # Rotate screen
        if time.ticks_diff(now, last_switch) >= SCREEN_DURATION_SEC * 1000:
            screen = (screen + 1) % 3
            last_switch = now
            if have_sample:
                render_screen(oled, screen, co2_f, t_f, rh_f, lvl)

        # Sensor poll
        if time.ticks_diff(now, last_ready) >= READY_POLL_MS:
            last_ready = now

            if scd.get_data_ready_status():
                co2, temp, rh = scd.read_measurement()

                co2_f = ema(co2_f, co2, EMA_CO2_ALPHA)
                t_f   = ema(t_f,   temp, EMA_T_ALPHA)
                rh_f  = ema(rh_f,  rh,  EMA_RH_ALPHA)

                prev_lvl = lvl
                lvl = update_level(lvl, co2_f)

                have_sample = True
                render_screen(oled, screen, co2_f, t_f, rh_f, lvl)

                print("CO2:%d ppm  T:%.2f C  RH:%.2f%%  LVL:%s" %
                      (int(co2_f), t_f, rh_f, level_name(lvl)))

                # Notify on upward transitions
                if wlan.isconnected() and lvl > prev_lvl and lvl >= 1:
                    msg = "Ventilate 🌬️ ({})\nCO2: {} ppm\nT: {:.1f} C\nRH: {:.1f} %".format(
                        level_name(lvl),
                        int(round(co2_f)),
                        float(t_f),
                        float(rh_f),
                    )
                    tg_send(msg)
                    last_remind = now

                # Reminders while still >= WARN
                if wlan.isconnected() and lvl >= 1 and time.ticks_diff(now, last_remind) > TG_REMIND_MS:
                    msg = "Reminder 🌬️ ({})\nCO2: {} ppm\nT: {:.1f} C\nRH: {:.1f} %".format(
                        level_name(lvl),
                        int(round(co2_f)),
                        float(t_f),
                        float(rh_f),
                    )
                    if tg_send(msg):
                        last_remind = now

        time.sleep_ms(20)

main()
