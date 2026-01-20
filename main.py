# from machine import Pin, I2C
# import time

# # ====== I2C pins (можешь поменять под свою плату) ======
# I2C_SDA = 8
# I2C_SCL = 9

# SCD4X_ADDR = 0x62

# def crc8(data: bytes) -> int:
#     # CRC-8 для Sensirion (polynomial 0x31, init 0xFF)
#     crc = 0xFF
#     for b in data:
#         crc ^= b
#         for _ in range(8):
#             if crc & 0x80:
#                 crc = ((crc << 1) & 0xFF) ^ 0x31
#             else:
#                 crc = (crc << 1) & 0xFF
#     return crc

# def build_cmd(cmd: int, args_words=None) -> bytes:
#     # cmd: 16-bit, args_words: list of 16-bit words
#     out = bytes([(cmd >> 8) & 0xFF, cmd & 0xFF])
#     if args_words:
#         for w in args_words:
#             msb = (w >> 8) & 0xFF
#             lsb = w & 0xFF
#             d = bytes([msb, lsb])
#             out += d + bytes([crc8(d)])
#     return out

# def parse_words_with_crc(buf: bytes):
#     # buffer: [MSB LSB CRC] * n
#     if len(buf) % 3 != 0:
#         raise ValueError("Bad length")
#     words = []
#     for i in range(0, len(buf), 3):
#         d = buf[i:i+2]
#         c = buf[i+2]
#         if crc8(d) != c:
#             raise ValueError("CRC mismatch")
#         words.append((d[0] << 8) | d[1])
#     return words

# class SCD41:
#     def __init__(self, i2c: I2C, addr=SCD4X_ADDR):
#         self.i2c = i2c
#         self.addr = addr

#     def _write_cmd(self, cmd: int, args_words=None):
#         self.i2c.writeto(self.addr, build_cmd(cmd, args_words))

#     def _read(self, nbytes: int) -> bytes:
#         return self.i2c.readfrom(self.addr, nbytes)

#     def stop_periodic_measurement(self):
#         self._write_cmd(0x3F86)
#         time.sleep_ms(500)

#     def start_periodic_measurement(self):
#         self._write_cmd(0x21B1)
#         time.sleep_ms(5)

#     def get_data_ready_status(self) -> bool:
#         self._write_cmd(0xE4B8)
#         time.sleep_ms(1)
#         buf = self._read(3)
#         status = parse_words_with_crc(buf)[0]
#         return (status & 0x07FF) != 0

#     def read_measurement(self):
#         self._write_cmd(0xEC05)
#         time.sleep_ms(1)
#         buf = self._read(9)
#         co2_raw, t_raw, rh_raw = parse_words_with_crc(buf)

#         co2 = co2_raw  # ppm

#         # Формулы из даташита Sensirion:
#         # T(°C) = -45 + 175 * (t_raw / 65535)
#         # RH(%) = 100 * (rh_raw / 65535)
#         temp = -45.0 + 175.0 * (t_raw / 65535.0)
#         rh = 100.0 * (rh_raw / 65535.0)
#         return co2, temp, rh

#     def get_serial_number(self):
#         self._write_cmd(0x3682)
#         time.sleep_ms(1)
#         buf = self._read(9)
#         w = parse_words_with_crc(buf)  # 3 words = 48-bit serial
#         serial = (w[0] << 32) | (w[1] << 16) | w[2]
#         return serial

# # ====== init ======
# i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=100_000)

# print("I2C scan:", [hex(a) for a in i2c.scan()])
# if SCD4X_ADDR not in i2c.scan():
#     raise RuntimeError("SCD41 not found on I2C address 0x62. Проверь проводку/pins.")

# scd = SCD41(i2c)

# # На всякий: остановим, потом стартанём измерения
# try:
#     scd.stop_periodic_measurement()
# except Exception:
#     pass

# print("SCD41 serial:", hex(scd.get_serial_number()))

# scd.start_periodic_measurement()
# print("Waiting for first measurement (обычно ~5 сек)...")

# while True:
#     try:
#         if scd.get_data_ready_status():
#             co2, temp, rh = scd.read_measurement()
#             print("CO2: %4d ppm | T: %5.2f C | RH: %5.2f %%"
#                   % (co2, temp, rh))
#         time.sleep(1)
#     except Exception as e:
#         print("Error:", e)
#         time.sleep(2)



from machine import Pin, I2C
import time
import sh1106

i2c = I2C(0, scl=Pin(12), sda=Pin(13), freq=400000)
oled = sh1106.SH1106_I2C(128, 64, i2c, addr=0x3C)

counter = 0
temperature = 23.5
status = "IDLE"

while True:
    # допустим, переменные меняются
    counter += 1
    temperature += 0.1
    status = "RUN" if (counter % 2 == 0) else "IDLE"

    # 1) очищаем экран
    oled.fill(0)

    # 2) рисуем актуальные значения
    oled.text("Counter: {}".format(counter), 0, 0)
    oled.text("Temp: {:.1f} C".format(temperature), 0, 16)
    oled.text("Status: {}".format(status), 0, 32)

    # 3) выводим
    oled.show()

    time.sleep(0.2)
