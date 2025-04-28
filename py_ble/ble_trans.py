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
        self.last_timestamp = time.time()
        self.start_time = self.last_timestamp
        self.total_bytes = 0
        self.byte_window = 0
        self.data_buffer = deque()

        # CSV
        self.file = open(self.csv_file, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["Time", "LED1", "Dark1", "LED2", "Dark2", "LED3", "Dark3", "LED4", "Dark4"])

    def log_data(self, timestamp, led_values):
        self.data_buffer.append([timestamp] + led_values)

        # if 50 lines write CSV
        if len(self.data_buffer) >= 50:
            self.flush_buffer()

    def calculate_bps(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        if elapsed_time >= 5.0:
            bps = (self.byte_window * 8) / elapsed_time
            print(f"Time: {current_time}, BLE Speed: {bps:.2f} bps")
            self.byte_window = 0
            self.start_time = current_time

    def update_byte_count(self, byte_count):
        self.total_bytes += byte_count
        self.byte_window += byte_count

    def flush_buffer(self):
        while self.data_buffer:
            self.writer.writerow(self.data_buffer.popleft())
        self.file.flush()

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
            print("No matching device found. You may need to edit match_nus_uuid().")
            sys.exit(1)

        def handle_disconnect(_: BleakClient):
            print("Device was disconnected. Goodbye.")
            for task in asyncio.all_tasks():
                task.cancel()

        def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
            logger.update_byte_count(len(data))
            if len(data) == 24:
                asyncio.create_task(data_queue.put(data)) # save asyncio

        async def process_data():
            while True:
                data = await data_queue.get()
                timestamp = time.time()

                led_values = []
                for i in range(0, 24, 6):
                    led = int.from_bytes(data[i:i + 3], byteorder="big", signed=True) / 8388608.0 * 4.0
                    dark = int.from_bytes(data[i + 3:i + 6], byteorder="big", signed=True) / 8388608.0 * 4.0
                    led_values.extend([led, dark])

                logger.log_data(timestamp, led_values)
                logger.calculate_bps()

        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

            # start data process
            asyncio.create_task(process_data())

            print("Connected. Receiving data...")

            # wait for data
            while True:
                await asyncio.sleep(1)

    finally:
        logger.close()


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        pass