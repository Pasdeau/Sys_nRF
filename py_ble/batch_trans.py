import asyncio
import csv
import sys
import time
from collections import deque

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class BLELogger:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.start_time = time.time()
        self.total_bytes = 0
        self.byte_window = 0
        self.data_buffer = deque()

        self.file = open(self.csv_file, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["Time", "LED1", "LED2", "LED3", "LED4", "Dark"])

    def log_data(self, timestamp, led_values):
        self.data_buffer.append([timestamp] + led_values[:4] + [led_values[-1]])  # 4 LEDs + 1 dark
        if len(self.data_buffer) >= 50:
            self.flush_buffer()

    def flush_buffer(self):
        while self.data_buffer:
            self.writer.writerow(self.data_buffer.popleft())
        self.file.flush()

    def update_byte_count(self, byte_count):
        self.total_bytes += byte_count
        self.byte_window += byte_count

    def calculate_bps(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        if elapsed_time >= 5.0:
            bps = (self.byte_window * 8) / elapsed_time
            print(f"Time: {current_time:.2f}, BLE Speed: {bps:.2f} bps")
            self.byte_window = 0
            self.start_time = current_time

    def close(self):
        self.flush_buffer()
        self.file.close()


async def uart_terminal():
    logger = BLELogger("test_data.csv")
    data_queue = asyncio.Queue()

    try:
        def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
            return UART_SERVICE_UUID.lower() in adv.service_uuids

        device = await BleakScanner.find_device_by_filter(match_nus_uuid)

        if device is None:
            print("No matching device found.")
            sys.exit(1)

        def handle_disconnect(_: BleakClient):
            print("Device disconnected.")
            for task in asyncio.all_tasks():
                task.cancel()

        def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
            logger.update_byte_count(len(data))
            if len(data) == 244:
                asyncio.create_task(data_queue.put(data))

        async def process_data():
            while True:
                data = await data_queue.get()

                # Step 1: Extract T0 (4 bytes, milliseconds)
                T0 = int.from_bytes(data[0:4], byteorder="big")

                # Step 2: Loop over 15 small packets
                for i in range(15):
                    offset = 4 + i * 16
                    delta = data[offset]  # 1-byte delta (ms)
                    timestamp = (T0 + delta) / 1000.0  # convert to seconds

                    led_values = []
                    for j in range(4):
                        base = offset + 1 + j * 3
                        raw = int.from_bytes(data[base:base + 3], byteorder="big", signed=True)
                        led = raw / 8388608.0 * 4.0
                        led_values.append(led)

                    # Dark (3 bytes at end)
                    dark_raw = int.from_bytes(data[offset + 13:offset + 16], byteorder="big", signed=True)
                    dark = dark_raw / 8388608.0 * 4.0
                    led_values.append(dark)

                    logger.log_data(timestamp, led_values)

                logger.calculate_bps()

        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            await client.start_notify(UART_TX_CHAR_UUID, handle_rx)
            asyncio.create_task(process_data())
            print("Connected. Receiving 244-byte data packets...")

            while True:
                await asyncio.sleep(1)

    finally:
        logger.close()


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        pass
