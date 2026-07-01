#!/usr/bin/env python3
import socket
import serial
import time
import sys


# ===================================================
# TCP SERVER AYARLARI
# ===================================================

HOST = "10.19.62.135"
PORT = 8888


# ===================================================
# ARDUINO SERIAL AYARLARI
# ===================================================

ARDUINO_BAUD = 115200


# ===================================================
# JOYSTICK AKTIF / PASIF AYARLARI
# ===================================================
# Bagli olmayan veya kullanmak istemedigin joysticki False yap.
#
# JOY1 = rover surus
# JOY2 = robot kol servo2 + servo3
# JOY3 = robot kol servo1 + servo4

ENABLE_JOY1_DRIVE = False
ENABLE_JOY2_ARM   = True
ENABLE_JOY3_ARM   = False


# ===================================================
# JOYSTICK AYARLARI
# ===================================================

JOY_CENTER = 2048
DEADZONE = 500
FAST_THRESHOLD = 1300

ESP_TIMEOUT = 0.7


def send_arduino(ser, command):
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()
    print("[ARDUINO]", command)


def direction_command(axis_value,
                      negative_cmd_slow,
                      negative_cmd_fast,
                      positive_cmd_slow,
                      positive_cmd_fast):
    diff = axis_value - JOY_CENTER

    if abs(diff) < DEADZONE:
        return None

    fast = abs(diff) > FAST_THRESHOLD

    if diff < 0:
        return negative_cmd_fast if fast else negative_cmd_slow
    else:
        return positive_cmd_fast if fast else positive_cmd_slow


def process_esp_line(line, ser, state):
    line = line.strip()

    if not line:
        return

    if line.startswith("ESP32"):
        print("[ESP]", line)
        return

    parts = line.split(",")

    if len(parts) != 11:
        print("[HATALI VERI]", line)
        return

    try:
        x1 = int(parts[0])
        y1 = int(parts[1])
        x2 = int(parts[2])
        y2 = int(parts[3])
        x3 = int(parts[4])
        y3 = int(parts[5])

        b4  = int(parts[6])
        b18 = int(parts[7])
        b19 = int(parts[8])
        b21 = int(parts[9])
        b27 = int(parts[10])

    except ValueError:
        print("[SAYIYA CEVRILEMEDI]", line)
        return

    print(f"[ESP DATA] x1={x1} y1={y1} | x2={x2} y2={y2} | x3={x3} y3={y3} | b4={b4} b18={b18} b19={b19} b21={b21} b27={b27}")

    # ===================================================
    # GPIO27 ACIL DUR
    # ===================================================
    # GPIO27 butonuna basilirsa rover ve sondaj durur.
    # Bu kisim her zaman aktif.

    if b27 == 1:
        send_arduino(ser, "dur")
        send_arduino(ser, "sondaj:dur")
        state["last_drive_cmd"] = "dur"
        state["last_drill_cmd"] = "sondaj:dur"
        print("[EMERGENCY] GPIO27 acil dur.")
        return

    # ===================================================
    # 1) JOY1 - ROVER SURUS
    # ===================================================

    if ENABLE_JOY1_DRIVE:
        drive_cmd = None

        # Oncelik ileri / geri
        if abs(y1 - JOY_CENTER) > DEADZONE:
            drive_cmd = direction_command(
                y1,
                "ileri_yavas", "ileri_hizli",   # y1 2048'den kucukse
                "geri_yavas",  "geri_hizli"     # y1 2048'den buyukse
            )

        elif abs(x1 - JOY_CENTER) > DEADZONE:
            drive_cmd = direction_command(
                x1,
                "sol_yavas", "sol_hizli",       # x1 2048'den kucukse
                "sag_yavas", "sag_hizli"        # x1 2048'den buyukse
            )

        else:
            drive_cmd = "dur"

    else:
        # JOY1 kapaliysa rover kesinlikle durur.
        drive_cmd = "dur"

    if drive_cmd != state["last_drive_cmd"]:
        send_arduino(ser, drive_cmd)
        state["last_drive_cmd"] = drive_cmd

    # ===================================================
    # 2) JOY2 - ROBOT KOL SERVO2 + SERVO3
    # ===================================================
    #
    # Arduino kodundaki eslesme:
    # x2:deger -> servo2
    # y2:deger -> servo3

    if ENABLE_JOY2_ARM:
        if abs(x2 - JOY_CENTER) > DEADZONE:
            send_arduino(ser, f"x2:{x2}")

        if abs(y2 - JOY_CENTER) > DEADZONE:
            send_arduino(ser, f"y2:{y2}")

    # ===================================================
    # 3) JOY3 - ROBOT KOL SERVO1 + SERVO4
    # ===================================================
    #
    # Arduino kodundaki eslesme:
    # x3:deger -> servo1
    # y3:deger -> servo4

    if ENABLE_JOY3_ARM:
        if abs(x3 - JOY_CENTER) > DEADZONE:
            send_arduino(ser, f"x3:{x3}")

        if abs(y3 - JOY_CENTER) > DEADZONE:
            send_arduino(ser, f"y3:{y3}")

    # ===================================================
    # 4) BUTONLAR
    # ===================================================
    #
    # GPIO4  -> servo5 yukari
    # GPIO18 -> servo5 asagi
    # GPIO19 -> sondaj yukari
    # GPIO21 -> sondaj asagi

    if b4 == 1:
        send_arduino(ser, "servo5:yukari")

    if b18 == 1:
        send_arduino(ser, "servo5:asagi")

    if b19 == 1:
        drill_cmd = "sondaj:yukari"
    elif b21 == 1:
        drill_cmd = "sondaj:asagi"
    else:
        drill_cmd = "sondaj:dur"

    if drill_cmd != state["last_drill_cmd"]:
        send_arduino(ser, drill_cmd)
        state["last_drill_cmd"] = drill_cmd


def main():
    arduino_port = "/dev/ttyACM0"

    print("[SERIAL] Arduino portu aciliyor:", arduino_port)

    try:
        ser = serial.Serial(arduino_port, ARDUINO_BAUD, timeout=0.1)
    except serial.SerialException as e:
        print("[HATA] Arduino seri port acilamadi.")
        print(e)
        sys.exit(1)

    print("[SERIAL] Arduino baglandi.")
    print("[SERIAL] Arduino reset bekleniyor...")
    time.sleep(2)

    state = {
        "last_drive_cmd": None,
        "last_drill_cmd": None,
    }

    # Baslangicta guvenlik
    send_arduino(ser, "dur")
    send_arduino(ser, "sondaj:dur")

    print()
    print("Aktif joystick ayarlari:")
    print("JOY1 DRIVE:", "AKTIF" if ENABLE_JOY1_DRIVE else "PASIF")
    print("JOY2 ARM  :", "AKTIF" if ENABLE_JOY2_ARM else "PASIF")
    print("JOY3 ARM  :", "AKTIF" if ENABLE_JOY3_ARM else "PASIF")
    print()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server.bind((HOST, PORT))
    server.listen(1)

    print("[TCP] Raspberry TCP server basladi.")
    print(f"[TCP] Dinlenen adres: {HOST}:{PORT}")
    print("[TCP] ESP32 baglantisi bekleniyor...")

    try:
        while True:
            conn, addr = server.accept()
            print()
            print("[TCP] ESP32 baglandi:", addr)

            conn.settimeout(0.2)
            buffer = ""
            last_data_time = time.time()

            state["last_drive_cmd"] = None
            state["last_drill_cmd"] = None

            send_arduino(ser, "dur")
            send_arduino(ser, "sondaj:dur")

            try:
                while True:
                    now = time.time()

                    try:
                        data = conn.recv(1024)
                    except socket.timeout:
                        data = b""

                    if data:
                        last_data_time = now
                        buffer += data.decode("utf-8", errors="ignore")

                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            process_esp_line(line, ser, state)

                    # ESP veri gondermeyi keserse guvenlik icin durdur.
                    if now - last_data_time > ESP_TIMEOUT:
                        if state["last_drive_cmd"] != "dur":
                            send_arduino(ser, "dur")
                            state["last_drive_cmd"] = "dur"

                        if state["last_drill_cmd"] != "sondaj:dur":
                            send_arduino(ser, "sondaj:dur")
                            state["last_drill_cmd"] = "sondaj:dur"

                    time.sleep(0.01)

            except ConnectionResetError:
                print("[TCP] ESP32 baglantisi koptu.")
                send_arduino(ser, "dur")
                send_arduino(ser, "sondaj:dur")

            finally:
                conn.close()
                print("[TCP] ESP32 yeniden bekleniyor...")

    except KeyboardInterrupt:
        print()
        print("[CIKIS] Program kapatiliyor...")
        send_arduino(ser, "dur")
        send_arduino(ser, "sondaj:dur")

    finally:
        ser.close()
        server.close()
        print("[CIKIS] Seri port ve TCP server kapatildi.")


if __name__ == "__main__":
    main()