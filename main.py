
# ESP32-S3 + SCD41 (I2C0 GPIO8/9) + SH1106 (I2C1 GPIO13/12)
# 3 screens: TEMP / CO2 / HUM
# Telegram alerts (outgoing) + HOME/AWAY via local HTTP (Shortcuts-friendly)
# Optional: also accept /home /away /status via Telegram polling (user -> bot)
#
# Notes:
# - This version fixes the "Sample age grows forever" issue by using a stall_deadline
#   (so restarts don't loop every READY_POLL_MS before the sensor has time to produce data).
# - Also adds basic try/except around I2C reads to avoid hard-stalls on occasional bus errors.

from machine import Pin, I2C
import time
import network
import socket
import framebuf
import sh1106

try:
    import urequests as requests
except ImportError:
    raise ImportError("urequests not found. Upload urequests.py to the board.")

try:
    import ujson as json
except ImportError:
    import json

try:
    import gc
except ImportError:
    gc = None


# ===================== USER SETTINGS =====================
# !!! IMPORTANT !!!
# Do NOT paste real secrets into shared code.
# Fill these in locally on your device.

WIFI_SSID = ""
WIFI_PASS = ""

TG_TOKEN  = ""  
TG_CHAT_ID =           # your private chat id with the bot (often == your user id)
TG_ALLOWED_USER_ID =   

# Home/Away default on boot
HOME_MODE_DEFAULT = True

# --- Local HTTP control (for Shortcuts) ---
HTTP_ENABLE = True
HTTP_PORT = 8080
HTTP_KEY = "MYKEY"  # set to something non-trivial

# Telegram: outgoing messages pacing
TG_MIN_GAP_MS = 1500
TG_REMIND_MS  = 20 * 60 * 1000  # reminder while >= WARN

# Telegram: optional command polling (user -> bot)
TG_CMDS_ENABLE = True
TG_CMD_POLL_MS = 6000

SEND_BOOT_MSG = True
WIFI_RECONNECT_MS = 10_000


# ===================== HARDWARE =====================
SCD4X_ADDR = 0x62
OLED_ADDR  = 0x3C

SCD_SDA  = 8
SCD_SCL  = 9
OLED_SDA = 13
OLED_SCL = 12

W, H = 128, 64

MEAS_MODE     = "normal"   # "normal" (~5s) or "lowpower" (~30s)
READY_POLL_MS = 350

ASC_ENABLED = True
ALTITUDE_M = 0

# How long we tolerate "no new sample" before restart measurement
SENSOR_STALL_MS_NORMAL   = 15_000
SENSOR_STALL_MS_LOWPOWER = 80_000

SCREEN_DURATION_SEC = 5

EMA_CO2_ALPHA = 0.35
EMA_T_ALPHA   = 0.20
EMA_RH_ALPHA  = 0.20

# CO2 levels with hysteresis
WARN_ON  = 1000
WARN_OFF = 850

HIGH_ON  = 1400
HIGH_OFF = 1200

RED_ON   = 2000
RED_OFF  = 1800

FORCE_TOFFSET_C = None
PERSIST_TOFFSET = False


# ===================== HELPERS =====================
def ema(prev, x, alpha):
    return x if prev is None else (prev + alpha * (x - prev))

def level_name(lvl):
    return ["OK", "WARN", "HIGH", "RED"][lvl]

def url_escape(s):
    out = []
    for b in s.encode("utf-8"):
        if (48 <= b <= 57) or (65 <= b <= 90) or (97 <= b <= 122) or b in b"-_.~":
            out.append(chr(b))
        elif b == 32:
            out.append("%20")
        else:
            out.append("%{:02X}".format(b))
    return "".join(out)

def safe_close(r):
    try:
        if r:
            r.close()
    except:
        pass


# ===================== WIFI =====================
def wifi_connect(timeout_ms=15000):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        return wlan

    try:
        wlan.connect(WIFI_SSID, WIFI_PASS)
    except Exception as e:
        print("WiFi connect error:", e)
        return wlan

    t0 = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            break
        time.sleep_ms(200)
    return wlan

def sync_time_ntp():
    try:
        import ntptime
        ntptime.settime()
        print("NTP time synced")
    except Exception as e:
        print("NTP sync skipped/error:", e)


# ===================== TELEGRAM (OUTGOING) =====================
_last_tg_send = 0

def tg_send(text):
    global _last_tg_send
    if (not TG_TOKEN) or TG_TOKEN.startswith("PUT_"):
        print("TG token not set")
        return False

    now = time.ticks_ms()
    if time.ticks_diff(now, _last_tg_send) < TG_MIN_GAP_MS:
        return False

    url = "https://api.telegram.org/bot{}/sendMessage?chat_id={}&text={}".format(
        TG_TOKEN, TG_CHAT_ID, url_escape(text)
    )

    r = None
    try:
        r = requests.get(url)
        code = getattr(r, "status_code", None)
        body = getattr(r, "text", "")
        if code is not None and code != 200:
            print("TG HTTP:", code, body[:200])
            return False
        _last_tg_send = now
        return True
    except Exception as e:
        print("TG send error:", e)
        return False
    finally:
        safe_close(r)


# ===================== TELEGRAM (OPTIONAL COMMANDS via getUpdates) =====================
_last_update_id = 0

def tg_get_updates(offset):
    url = "https://api.telegram.org/bot{}/getUpdates?timeout=0&offset={}&limit=5".format(
        TG_TOKEN, offset
    )
    r = None
    try:
        r = requests.get(url)
        try:
            return r.json()
        except:
            return json.loads(r.text)
    except Exception as e:
        print("TG getUpdates error:", e)
        return None
    finally:
        safe_close(r)

def tg_poll_commands(set_home_cb, set_away_cb, status_cb):
    global _last_update_id
    if (not TG_CMDS_ENABLE) or (not TG_TOKEN) or TG_TOKEN.startswith("PUT_"):
        return

    data = tg_get_updates(_last_update_id + 1)
    if not data or not data.get("ok"):
        return

    for upd in data.get("result", []):
        uid = upd.get("update_id", 0)
        if uid > _last_update_id:
            _last_update_id = uid

        msg = upd.get("message")
        if not msg:
            continue

        from_id = (msg.get("from") or {}).get("id")
        if from_id != TG_ALLOWED_USER_ID:
            continue

        text = (msg.get("text") or "").strip().lower()
        cmd = text.split()[0] if text else ""
        if cmd.startswith("/"):
            cmd = cmd[1:]

        if cmd == "home":
            set_home_cb()
            tg_send("Home mode ON ✅")
        elif cmd == "away":
            set_away_cb()
            tg_send("Away mode ON 💤")
        elif cmd in ("status", "start", "help"):
            tg_send(status_cb())


# ===================== LOCAL HTTP CONTROL (Shortcuts) =====================
def http_setup():
    if not HTTP_ENABLE:
        return None
    addr = socket.getaddrinfo("0.0.0.0", HTTP_PORT)[0][-1]
    s = socket.socket()
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except:
        pass
    s.bind(addr)
    s.listen(2)
    s.settimeout(0)  # non-blocking accept
    print("HTTP server listening on port", HTTP_PORT)
    return s

def _http_reply(cl, code=200, body="OK", content_type="text/plain"):
    try:
        hdr = "HTTP/1.1 {} OK\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n".format(
            code, content_type, len(body)
        )
        cl.send(hdr.encode())
        cl.send(body.encode())
    except:
        pass

def http_poll(http_sock, set_home_cb, set_away_cb, status_cb):
    if not http_sock:
        return

    try:
        cl, _ = http_sock.accept()
    except OSError:
        return

    try:
        cl.settimeout(0.3)
        req = cl.recv(512)
        if not req:
            cl.close()
            return

        line = req.split(b"\r\n", 1)[0].decode("utf-8", "ignore")
        parts = line.split(" ")
        path = parts[1] if len(parts) >= 2 else "/"

        path_only, _, query = path.partition("?")
        params = {}
        if query:
            for kv in query.split("&"):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    params[k] = v

        if HTTP_KEY and params.get("key") != HTTP_KEY:
            _http_reply(cl, 403, "Forbidden")
            cl.close()
            return

        if path_only == "/home":
            set_home_cb()
            _http_reply(cl, 200, "HOME")
        elif path_only == "/away":
            set_away_cb()
            _http_reply(cl, 200, "AWAY")
        elif path_only == "/status":
            _http_reply(cl, 200, status_cb(), "text/plain")
        else:
            _http_reply(cl, 200, "Endpoints: /home /away /status")
    except Exception:
        pass
    finally:
        try:
            cl.close()
        except:
            pass


# ===================== SENSIRION CRC =====================
def crc8(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x31
            else:
                crc = (crc << 1) & 0xFF
    return crc

def build_cmd(cmd, args_words=None):
    out = bytes([(cmd >> 8) & 0xFF, cmd & 0xFF])
    if args_words:
        for w in args_words:
            d = bytes([(w >> 8) & 0xFF, w & 0xFF])
            out += d + bytes([crc8(d)])
    return out

def parse_words_with_crc(buf):
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


# ===================== SCD41 =====================
class SCD41:
    def __init__(self, i2c, addr=SCD4X_ADDR):
        self.i2c = i2c
        self.addr = addr

    def _write_cmd(self, cmd, args_words=None):
        self.i2c.writeto(self.addr, build_cmd(cmd, args_words))

    def _read_words(self, nwords):
        buf = self.i2c.readfrom(self.addr, nwords * 3)
        return parse_words_with_crc(buf)

    def stop_periodic_measurement(self):
        self._write_cmd(0x3F86)
        time.sleep_ms(500)

    def start_periodic_measurement(self):
        self._write_cmd(0x21B1)
        time.sleep_ms(5)

    def start_low_power_periodic_measurement(self):
        self._write_cmd(0x21AC)
        time.sleep_ms(5)

    def get_data_ready_status(self):
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

    def set_asc_enabled(self, enabled):
        self._write_cmd(0x2416, [1 if enabled else 0])
        time.sleep_ms(1)

    def set_sensor_altitude(self, altitude_m):
        altitude_m = int(max(0, min(3000, altitude_m)))
        self._write_cmd(0x2427, [altitude_m])
        time.sleep_ms(1)

    def persist_settings(self):
        self._write_cmd(0x3615)
        time.sleep_ms(800)

    def set_temperature_offset(self, offset_c):
        offset_c = float(offset_c)
        if offset_c < 0:
            offset_c = 0.0
        if offset_c > 20:
            offset_c = 20.0
        word = int(round(offset_c * 65535.0 / 175.0))
        self._write_cmd(0x241D, [word])
        time.sleep_ms(1)


# ===================== OLED =====================
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
                bx = x + xx * scale
                by = y + yy * scale
                if bx >= W or by >= H:
                    continue
                for dy in range(scale):
                    py = by + dy
                    if 0 <= py < H:
                        for dx in range(scale):
                            px = bx + dx
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

    bw2 = len(big) * 8 * sb
    dw2 = len(dec) * 8 * sd
    x0 = (W - (bw2 + dw2 + 2)) // 2
    yb = 20
    yd = yb + (sb*8 - sd*8)

    text_scaled(oled, big, x0, yb, sb)
    text_scaled(oled, dec, x0 + bw2 + 2, yd, sd)

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


# ===================== CO2 LEVEL FSM =====================
def update_level(prev, co2):
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

    if co2 <= RED_OFF:
        if co2 >= HIGH_ON: return 2
        if co2 >= WARN_ON: return 1
        return 0
    return 3


# ===================== MAIN =====================
def main():
    HOME_MODE = HOME_MODE_DEFAULT

    def set_home():
        nonlocal HOME_MODE
        HOME_MODE = True
        print("HOME_MODE = True")

    def set_away():
        nonlocal HOME_MODE
        HOME_MODE = False
        print("HOME_MODE = False")

    # Wi-Fi
    wlan = wifi_connect()
    ip = wlan.ifconfig()[0] if wlan.isconnected() else "0.0.0.0"
    print("Wi-Fi:", wlan.isconnected(), wlan.ifconfig() if wlan.isconnected() else None)

    if wlan.isconnected():
        sync_time_ntp()

    # HTTP server for Shortcuts
    http_sock = None
    if wlan.isconnected() and HTTP_ENABLE:
        try:
            http_sock = http_setup()
            print("Shortcuts URLs:")
            print("  http://{}:{}/home?key={}".format(ip, HTTP_PORT, HTTP_KEY))
            print("  http://{}:{}/away?key={}".format(ip, HTTP_PORT, HTTP_KEY))
            print("  http://{}:{}/status?key={}".format(ip, HTTP_PORT, HTTP_KEY))
        except Exception as e:
            print("HTTP setup error:", e)
            http_sock = None

    # I2C + OLED
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

    try:
        scd.set_asc_enabled(ASC_ENABLED)
        scd.set_sensor_altitude(ALTITUDE_M)
    except Exception as e:
        print("SCD init settings error:", e)

    if FORCE_TOFFSET_C is not None:
        try:
            scd.set_temperature_offset(FORCE_TOFFSET_C)
            time.sleep_ms(10)
            if PERSIST_TOFFSET:
                scd.persist_settings()
        except Exception as e:
            print("Temp offset error:", e)

    def start_measurement():
        if MEAS_MODE == "lowpower":
            scd.start_low_power_periodic_measurement()
            return SENSOR_STALL_MS_LOWPOWER
        scd.start_periodic_measurement()
        return SENSOR_STALL_MS_NORMAL

    def restart_measurement(reason=""):
        nonlocal stall_ms, stall_deadline, sensor_errs
        print("Restart measurement", ("(" + reason + ")") if reason else "")
        try:
            scd.stop_periodic_measurement()
        except:
            pass
        time.sleep_ms(200)
        stall_ms = start_measurement()
        stall_deadline = time.ticks_add(time.ticks_ms(), stall_ms)
        sensor_errs = 0

    stall_ms = start_measurement()
    stall_deadline = time.ticks_add(time.ticks_ms(), stall_ms)

    # runtime state
    screen = 0
    last_switch = time.ticks_ms()
    last_ready  = time.ticks_ms()
    last_wifi   = time.ticks_ms()
    last_cmd    = time.ticks_ms()
    last_remind = time.ticks_add(time.ticks_ms(), -TG_REMIND_MS)

    co2_f = None
    t_f   = None
    rh_f  = None
    lvl   = 0
    have_sample = False
    last_sample_ms = None

    sensor_errs = 0
    SENSOR_ERRS_BEFORE_RESTART = 3

    def status_text():
        cur_ip = wlan.ifconfig()[0] if wlan.isconnected() else ip
        age_s = "-"
        if last_sample_ms is not None:
            age_s = str(time.ticks_diff(time.ticks_ms(), last_sample_ms) // 1000)

        return (
            "Mode: {}\n"
            "CO2: {} ppm ({})\n"
            "T: {} C\n"
            "RH: {} %\n"
            "Sample age: {} s\n"
            "HTTP: http://{}:{}/status?key={}"
        ).format(
            "HOME" if HOME_MODE else "AWAY",
            "-" if co2_f is None else int(round(co2_f)),
            "-" if not have_sample else level_name(lvl),
            "-" if t_f is None else "{:.1f}".format(t_f),
            "-" if rh_f is None else "{:.1f}".format(rh_f),
            age_s,
            cur_ip, HTTP_PORT, HTTP_KEY
        )

    # boot message
    if SEND_BOOT_MSG and wlan.isconnected():
        tg_send("CO2 monitor booted ✅\nMode: {}".format("HOME" if HOME_MODE else "AWAY"))

    render_screen(oled, screen, 400.0, 0.0, 0.0, lvl)

    while True:
        now = time.ticks_ms()

        # handle HTTP control (Shortcuts)
        if http_sock:
            http_poll(http_sock, set_home, set_away, status_text)

        # Wi-Fi watchdog / reconnect
        if time.ticks_diff(now, last_wifi) > WIFI_RECONNECT_MS:
            last_wifi = now
            if not wlan.isconnected():
                wlan = wifi_connect()
                if wlan.isconnected():
                    ip = wlan.ifconfig()[0]
                    sync_time_ntp()
                    # if HTTP was not running, start it
                    if HTTP_ENABLE and (http_sock is None):
                        try:
                            http_sock = http_setup()
                            print("Shortcuts URLs:")
                            print("  http://{}:{}/home?key={}".format(ip, HTTP_PORT, HTTP_KEY))
                            print("  http://{}:{}/away?key={}".format(ip, HTTP_PORT, HTTP_KEY))
                            print("  http://{}:{}/status?key={}".format(ip, HTTP_PORT, HTTP_KEY))
                        except Exception as e:
                            print("HTTP setup error:", e)
                            http_sock = None

        # Optional: Telegram command polling
        if wlan.isconnected() and TG_CMDS_ENABLE and time.ticks_diff(now, last_cmd) > TG_CMD_POLL_MS:
            last_cmd = now
            tg_poll_commands(set_home, set_away, status_text)

        # screen rotation
        if time.ticks_diff(now, last_switch) >= SCREEN_DURATION_SEC * 1000:
            screen = (screen + 1) % 3
            last_switch = now
            if have_sample:
                render_screen(oled, screen, co2_f, t_f, rh_f, lvl)

        # sensor poll
        if time.ticks_diff(now, last_ready) >= READY_POLL_MS:
            last_ready = now

            # stall watchdog (FIXED: uses stall_deadline so it doesn't restart in a tight loop)
            if time.ticks_diff(now, stall_deadline) > 0:
                restart_measurement("stall deadline exceeded")

            # data ready?
            try:
                ready = scd.get_data_ready_status()
            except Exception as e:
                sensor_errs += 1
                print("get_data_ready_status error:", e, "errs:", sensor_errs)
                if sensor_errs >= SENSOR_ERRS_BEFORE_RESTART:
                    restart_measurement("ready_status errors")
                ready = False

            if ready:
                try:
                    co2, temp, rh = scd.read_measurement()
                    sensor_errs = 0
                except Exception as e:
                    sensor_errs += 1
                    print("read_measurement error:", e, "errs:", sensor_errs)
                    if sensor_errs >= SENSOR_ERRS_BEFORE_RESTART:
                        restart_measurement("read errors")
                    co2 = temp = rh = None

                if co2 is not None:
                    co2_f = ema(co2_f, co2, EMA_CO2_ALPHA)
                    t_f   = ema(t_f,   temp, EMA_T_ALPHA)
                    rh_f  = ema(rh_f,  rh,  EMA_RH_ALPHA)

                    prev_lvl = lvl
                    lvl = update_level(lvl, co2_f)

                    have_sample = True
                    last_sample_ms = now

                    # every time we get a sample, push the stall deadline forward
                    stall_deadline = time.ticks_add(now, stall_ms)

                    render_screen(oled, screen, co2_f, t_f, rh_f, lvl)

                    print("CO2:%d  T:%.2f  RH:%.2f  LVL:%s  MODE:%s" %
                          (int(round(co2_f)), t_f, rh_f, level_name(lvl), ("HOME" if HOME_MODE else "AWAY")))

                    # Telegram alerts only in HOME mode
                    if wlan.isconnected() and HOME_MODE:
                        if lvl > prev_lvl and lvl >= 1:
                            msg = "Ventilate 🌬️ ({})\nCO2: {} ppm\nT: {:.1f} C\nRH: {:.1f} %".format(
                                level_name(lvl),
                                int(round(co2_f)),
                                float(t_f),
                                float(rh_f),
                            )
                            if tg_send(msg):
                                last_remind = now

                        if lvl >= 1 and time.ticks_diff(now, last_remind) > TG_REMIND_MS:
                            msg = "Reminder 🌬️ ({})\nCO2: {} ppm\nT: {:.1f} C\nRH: {:.1f} %".format(
                                level_name(lvl),
                                int(round(co2_f)),
                                float(t_f),
                                float(rh_f),
                            )
                            if tg_send(msg):
                                last_remind = now

        # GC occasionally
        if gc and (now % 30000 < 30):
            gc.collect()

        time.sleep_ms(20)

main()
