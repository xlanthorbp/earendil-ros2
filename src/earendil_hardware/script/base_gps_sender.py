#!/usr/bin/env python3
"""
Base Station GPS Sender
------------------------
Runs on your LAPTOP. Reads GPS from a USB module,
broadcasts the coordinates over WiFi (UDP) so the rover can find you.

Usage:
    python3 base_gps_sender.py              # Default port: /dev/ttyUSB0
    python3 base_gps_sender.py /dev/ttyACM0 # Custom port

The rover's gps_nav_test.py listens for these broadcasts automatically.
No IP address configuration needed — it uses UDP broadcast.
"""
import serial
import socket
import sys
import time

# ============================================
# CONFIGURATION
# ============================================
GPS_PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
GPS_BAUD = 9600
UDP_PORT = 5555          # Rover listens on this port
BROADCAST_RATE = 1.0     # Send once per second

def parse_nmea_coord(raw, direction):
    """Convert NMEA coordinate (DDMM.MMMM) to decimal degrees."""
    if not raw:
        return None
    # NMEA format: DDMM.MMMM (lat) or DDDMM.MMMM (lon)
    if direction in ('N', 'S'):
        degrees = float(raw[:2])
        minutes = float(raw[2:])
    else:  # E, W
        degrees = float(raw[:3])
        minutes = float(raw[3:])
    decimal = degrees + minutes / 60.0
    if direction in ('S', 'W'):
        decimal = -decimal
    return decimal

def parse_gga(parts):
    """Parse $GPGGA or $GNGGA sentence for lat/lon."""
    try:
        lat = parse_nmea_coord(parts[2], parts[3])
        lon = parse_nmea_coord(parts[4], parts[5])
        if lat and lon:
            return lat, lon
    except (IndexError, ValueError):
        pass
    return None, None

def main():
    # Setup UDP broadcast socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"=== BASE STATION GPS SENDER ===")
    print(f"GPS Port : {GPS_PORT}")
    print(f"Baud     : {GPS_BAUD}")
    print(f"UDP Port : {UDP_PORT}")
    print(f"Broadcasting to all devices on the network...")
    print()

    try:
        gps_serial = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
        print(f"[OK] GPS connected on {GPS_PORT}")
    except Exception as e:
        print(f"[ERROR] Cannot open GPS port {GPS_PORT}: {e}")
        print(f"  TIP: Try:  python3 base_gps_sender.py /dev/ttyACM0")
        print(f"  TIP: Check: ls /dev/ttyUSB* /dev/ttyACM*")
        sys.exit(1)

    last_send = 0

    while True:
        try:
            line = gps_serial.readline().decode('ascii', errors='ignore').strip()

            # Look for GGA sentences (most reliable for position)
            if '$G' in line and 'GGA' in line:
                parts = line.split(',')
                lat, lon = parse_gga(parts)

                if lat is not None and lon is not None:
                    now = time.time()
                    if now - last_send >= BROADCAST_RATE:
                        message = f"BASE,{lat:.8f},{lon:.8f}\n"
                        sock.sendto(message.encode(), ('<broadcast>', UDP_PORT))
                        print(f"  [SENT] Lat: {lat:.6f}  Lon: {lon:.6f}")
                        last_send = now

        except KeyboardInterrupt:
            print("\nStopped.")
            break
        except Exception as e:
            print(f"  [WARN] {e}")
            time.sleep(0.5)

    sock.close()
    gps_serial.close()

if __name__ == '__main__':
    main()
