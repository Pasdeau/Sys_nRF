import asyncio
import csv
import sys
import time
from itertools import count, takewhile
from typing import Iterator
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
        self.byte_buffer = bytearray()  # 新增缓冲区用于处理不完整数据

        self.file = open(self.csv_file, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["Time", "Value"])  # 修改CSV标题

    def log_data(self, timestamp, value):
        self.data_buffer.append([timestamp, value])

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
        if len(self.byte_buffer) > 0:
            print(f"Warning: {len(self.byte_buffer)} bytes remaining in buffer")
        self.file.close()


def sliced(data: bytes, n: int) -> Iterator[bytes]:
    return takewhile(len, (data[i: i + n] for i in count(0, n)))


async def uart_terminal():
    logger = BLELogger("test_mono.csv")
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

            # 将新数据添加到缓冲区
            logger.byte_buffer.extend(data)

            # 处理所有完整的3字节块
            while len(logger.byte_buffer) >= 3:
                chunk = logger.byte_buffer[:3]
                del logger.byte_buffer[:3]

                # 将三个字节转换为整数（dummy[0]为MSB，dummy[2]为LSB）
                value = int.from_bytes(chunk, byteorder='big')
                timestamp = time.time()
                logger.log_data(timestamp, value)

            logger.calculate_bps()

        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

            print("Connected. Start typing and press ENTER to send data...")

            loop = asyncio.get_running_loop()
            nus = client.services.get_service(UART_SERVICE_UUID)
            rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

            while True:
                data = await loop.run_in_executor(None, sys.stdin.buffer.readline)
                if not data:
                    break
                for s in sliced(data, rx_char.max_write_without_response_size):
                    await client.write_gatt_char(rx_char, s)
                print("Sent:", data)
    finally:
        logger.close()


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        pass