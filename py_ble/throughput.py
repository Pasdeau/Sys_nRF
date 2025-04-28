import asyncio
import time
from bleak import BleakScanner, BleakClient

THROUGHPUT_SERVICE_UUID = "0483DADD-6C9D-6CA9-5D41-03AD4FFF4ABB"
WRITE_CHARACTERISTIC_UUID = "1524"
PHY_CHARACTERISTIC_UUID = "1801"
DATA_251_BYTES = bytes(range(1))
TEST_DURATION = 1  # Test duration in seconds
BLE_PACKET_INTERVAL = 0.0005  # time delay per packet to simulate real BLE transmission timing

async def find_nrf5340():
    print("Scanning for nRF5340...")
    devices = await BleakScanner.discover()

    for device in devices:
        if device.name and "Nordic" in device.name:
            print(f"Found nRF5340: {device.name} [{device.address}]")
            return device.address

    print("No nRF5340 device found. Ensure it is advertising.")
    return None

async def get_phy_mode(client):
    """ Try to retrieve the PHY mode from the device """
    try:
        conn_info = await client.read_gatt_char(PHY_CHARACTERISTIC_UUID)
        phy_mode = conn_info[0]  # Assuming single-byte response
        print(f"PHY Mode: {'2M' if phy_mode == 2 else '1M' if phy_mode == 1 else 'Unknown'}")
    except Exception:
        print("Could not retrieve PHY mode. Check nRF log.")

async def test_ble_throughput(address):
    async with BleakClient(address) as client:
        print(f"Connected to {address}")

        services = await client.get_services()
        found_service = any(service.uuid.lower() == THROUGHPUT_SERVICE_UUID.lower() for service in services)

        if not found_service:
            print("Throughput Service not found.")
            return

        print(f"Throughput Service found: {THROUGHPUT_SERVICE_UUID}")

        # Retrieve PHY mode (if implemented on nRF side)
        await get_phy_mode(client)

        start_time = time.time()
        bytes_sent = 0

        print(f"Starting BLE throughput test for {TEST_DURATION} seconds...")

        while (time.time() - start_time) < TEST_DURATION:
            await client.write_gatt_char(WRITE_CHARACTERISTIC_UUID, DATA_251_BYTES, response=False)
            bytes_sent += len(DATA_251_BYTES)
            await asyncio.sleep(BLE_PACKET_INTERVAL)  # Ensure real-time BLE constraints

        duration = time.time() - start_time
        throughput_kbps = (bytes_sent * 8) / (duration * 1000)

        print("\nTransfer complete")
        print(f"Total data sent: {bytes_sent} bytes")
        print(f"Time elapsed: {duration:.5f} sec")
        print(f"Throughput: {throughput_kbps:.0f} kbps")

async def main():
    address = await find_nrf5340()
    if address:
        await test_ble_throughput(address)

if __name__ == "__main__":
    asyncio.run(main())