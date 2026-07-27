#!/usr/bin/env python3
import argparse
import serial
import time
from collections import Counter

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

def checksum(body: str) -> str:
    value = 0
    for ch in body:
        value ^= ord(ch)
    return f"{value:02X}"

def packet(body: str) -> bytes:
    return f"${body}*{checksum(body)}\r\n".encode("ascii")

def send_command(ser: serial.Serial, body: str, wait_s: float = 0.8) -> bytes:
    data = packet(body)
    print(f">>> {data.decode().strip()}")
    ser.write(data)
    ser.flush()
    time.sleep(wait_s)
    reply = ser.read(8192)
    if reply:
        text = reply.decode("ascii", errors="replace")
        print(text)
    else:
        print("Cevap yok")
    return reply

def extract_rtcm_ids(data: bytes) -> Counter:
    counts = Counter()
    buf = bytearray(data)

    while True:
        idx = buf.find(b"\xD3")
        if idx < 0:
            break
        if idx:
            del buf[:idx]
        if len(buf) < 3:
            break

        if buf[1] & 0xFC:
            del buf[0]
            continue

        length = ((buf[1] & 0x03) << 8) | buf[2]
        total = 3 + length + 3
        if total < 6 or total > 1030:
            del buf[0]
            continue
        if len(buf) < total:
            break

        frame = bytes(buf[:total])
        del buf[:total]
        payload = frame[3:3 + length]
        if len(payload) >= 2:
            msg_id = (payload[0] << 4) | (payload[1] >> 4)
            counts[msg_id] += 1

    return counts

def configure(port: str, baud: int, survey_s: int, survey_acc_m: float) -> None:
    commands = [
        "PQTMCFGRCVRMODE,W,2",
        "PQTMSAVEPAR",
        "PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1",
        f"PQTMCFGSVIN,W,1,{survey_s},{survey_acc_m},0,0,0",
        "PAIR432,0",
        "PAIR434,1",
        "PAIR436,1",
        "PAIR062,0,1",
        "PQTMSAVEPAR",
    ]

    print(f"[BASE] GPS: {port} @ {baud}")
    with serial.Serial(port, baud, timeout=0.25) as ser:
        time.sleep(1)
        ser.reset_input_buffer()
        for body in commands:
            send_command(ser, body)

    print("\n[BASE] Ayarlar kaydedildi.")
    print("[BASE] GPS'i kapat, iki anahtarı UART konumuna al,")
    print("[BASE] GPS TX -> Base RF RXI bağlantısını yap ve sistemi yeniden besle.")
    print("[BASE] Açık gökyüzünde base antenini sabit bırak.")

def check(port: str, baud: int, seconds: int) -> None:
    print(f"[BASE CHECK] {port} @ {baud}, {seconds} saniye")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        end = time.time() + seconds
        data = bytearray()
        while time.time() < end:
            chunk = ser.read(4096)
            if chunk:
                data.extend(chunk)

    counts = extract_rtcm_ids(bytes(data))
    print("Byte:", len(data))
    print("Raw D3:", data.count(b"\xD3"))
    print("GGA:", data.count(b"GGA"))
    print("RMC:", data.count(b"RMC"))
    print("RTCM mesajları:", dict(sorted(counts.items())) if counts else "yok")

    text = data.decode("ascii", errors="ignore")
    for line in text.splitlines():
        if "GNGGA" in line or "PQTMSVINSTATUS" in line:
            print(line)

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--survey-s", type=int, default=600)
    parser.add_argument("--survey-acc", type=float, default=0)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--check-seconds", type=int, default=10)
    args = parser.parse_args()

    if args.check_only:
        check(args.port, args.baud, args.check_seconds)
    else:
        configure(args.port, args.baud, args.survey_s, args.survey_acc)

if __name__ == "__main__":
    main()