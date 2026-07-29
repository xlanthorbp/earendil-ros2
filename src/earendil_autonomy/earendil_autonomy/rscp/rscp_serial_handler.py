import time
import threading
import serial
from cobs import cobs

from earendil_autonomy.rscp.rscp_protobuf.rscp_protobuf import RequestEnvelope, ResponseEnvelope


class RSCPSerialHandler:
    """
    RSCP COBS ve Protobuf Seri Port Sürücüsü.
    RS-232 seri portu üzerinden COBS kodlanmış Protobuf paketlerini okur ve basar.
    """

    def __init__(self, port='/dev/ttyUSB2', baudrate=115200, on_request_cb=None, logger=None):
        self.port = port
        self.baudrate = baudrate
        self.on_request_cb = on_request_cb
        self.logger = logger

        self.ser = None
        self.running = False
        self.read_thread = None
        self.rx_buffer = bytearray()
        self.lock = threading.RLock()

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            if self.logger:
                self.logger.info(f"[RSCP SERIAL] Connected to {self.port} @ {self.baudrate}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"[RSCP SERIAL] Failed to open serial port {self.port}: {e}")
            self.ser = None

    def stop(self):
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        if self.logger:
            self.logger.info("[RSCP SERIAL] Disconnected.")

    def _read_loop(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.5)
                continue

            try:
                data = self.ser.read(1024)
                if data:
                    with self.lock:
                        self.rx_buffer.extend(data)
                        self._process_buffer()
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[RSCP SERIAL] Read error: {e}")
                time.sleep(0.1)

    def _process_buffer(self):
        # COBS framing Uses 0x00 byte as packet delimiter
        while b'\x00' in self.rx_buffer:
            delimiter_idx = self.rx_buffer.index(b'\x00')
            raw_frame = bytes(self.rx_buffer[:delimiter_idx])
            del self.rx_buffer[:delimiter_idx + 1]

            if not raw_frame:
                continue

            try:
                decoded_bytes = cobs.decode(raw_frame)
                req = RequestEnvelope()
                req.ParseFromString(decoded_bytes)

                if self.on_request_cb:
                    self.on_request_cb(req)

            except Exception as e:
                if self.logger:
                    self.logger.warn(f"[RSCP SERIAL] Frame decode/parse error: {e}")

    def send_response(self, response_envelope: ResponseEnvelope):
        if not self.ser or not self.ser.is_open:
            if self.logger:
                self.logger.warn("[RSCP SERIAL] Cannot send response, serial port not open.")
            return False

        try:
            serialized_bytes = response_envelope.SerializeToString()
            encoded_bytes = cobs.encode(serialized_bytes)
            packet = encoded_bytes + b'\x00'

            with self.lock:
                self.ser.write(packet)
                self.ser.flush()
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"[RSCP SERIAL] Send error: {e}")
            return False
