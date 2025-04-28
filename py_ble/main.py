import asyncio
import csv
import math
import sys
import time
from itertools import count, takewhile
from typing import Iterator

import numpy as np
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from scipy.signal import butter, lfilter
from scipy.signal import savgol_filter

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

dbt = 0
bpm = 0
chronobpm = 10
chronoecg = 10
bpm_ecg = 0


# TIP: you can get this function and more from the ``more-itertools`` package.
def sliced(data: bytes, n: int) -> Iterator[bytes]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i: i + n] for i in count(0, n)))


# init parameters
d = np.float32((4.32 / 10) + 1)  # Distance entre la led et le photo recepteur
e_hb02_l1 = np.float32(464.5)
e_hb02_l2 = np.float32(982.6)
e_hb02_l3 = np.float32(334.5)
e_hb02_l4 = np.float32(1313.4)

e_hb_l1 = np.float32(1295.6)
e_hb_l2 = np.float32(950.9)
e_hb_l3 = np.float32(3439.9)
e_hb_l4 = np.float32(866.8)

e_methb_l1 = np.float32(168.8)
e_methb_l2 = np.float32(795.1)
e_methb_l3 = np.float32(609.4)
e_methb_l4 = np.float32(890.9)

e_cco_l1 = np.float32(1800.5)
e_cco_l2 = np.float32(1176.1)
e_cco_l3 = np.float32(4399.8)
e_cco_l4 = np.float32(1958.9)

dpf_t_l1 = np.float32(3.49)
dpf_t_l2 = np.float32(3.25)
dpf_t_l3 = np.float32(2.80)
dpf_t_l4 = np.float32(2.00)

matrice_data = np.array([[e_hb02_l1, e_hb_l1, e_methb_l1, e_cco_l1], [e_hb02_l2, e_hb_l2, e_methb_l2, e_cco_l2],
                         [e_hb02_l3, e_hb_l3, e_methb_l3, e_cco_l3], [e_hb02_l4, e_hb_l4, e_methb_l4, e_cco_l4]],
                        dtype=np.float32)


def calc_spo2(l1, l2, l3, l4):
    mesure = np.array([[l1 / dpf_t_l1], [l2 / dpf_t_l2], [l3 / dpf_t_l3], [l4 / dpf_t_l4]], dtype=np.float32)
    matricemesure = np.matmul(matrice_data, mesure)
    resultat = (1 / d) * matricemesure
    spo2 = 100 * (resultat[0, 0] / (resultat[0, 0] + resultat[1, 0] + resultat[2, 0]))
    return spo2, resultat


cnt = 0  # Compteur du nombre de points
beg = 0  # Début du chronomètre
spo2 = 95


async def uart_terminal():
    """
    This is a simple "terminal" program that uses the Nordic Semiconductor
    (nRF) UART service. It reads from stdin and sends each line of data to the
    remote device. Any data received from the device is printed to stdout.
    """

    temps = [0]  # Liste des temps
    LED1 = [1.5]
    LED2 = [1.5]
    LED3 = [1.5]
    LED4 = [1.5]
    REC1 = [1.5]
    REC2 = [1.5]
    REC3 = [1.5]
    REC4 = [1.5]
    LISTEECG = [1.5]
    with open("data.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            ["TEMPS", "LED1", "LED2", "LED3", "LED4", "Delta HbO2", "Delta Hb", "Delta MetHb", "Delta CCO", "SPO2",
             "Delta SPO2", "ECG"])
        # writer.writerow(["TEMPS", "ECG"])
        file.flush()

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        global beg
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            beg = time.time()  # Début du chronomètre
            return True

        return False

    device = await BleakScanner.find_device_by_filter(match_nus_uuid)

    if device is None:
        print("no matching device found, you may need to edit match_nus_uuid().")
        sys.exit(1)

    def handle_disconnect(_: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()
        file.close()

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):

        global cnt
        global beg
        global spo2
        global chronobpm
        global bpm
        global dbt
        global flag1
        global start
        global chronoecg
        global bpm_ecg

        heart_rate_span = [10, 250]  # max span of heart rate
        pts = 1800  # points used for peak finding (400 Hz, I recommend at least 4s (1600 pts)
        smoothing_size = 20  # convolution smoothing size
        t_vec, y_vals = [], []
        # print("received:", data)
        # delete bytearray from data
        # s = data.replace('bytearray(b'.encode(),''.encode())
        # print(data[0:3])
        # print(data[3:6])
        a = int((data[0:3].hex()), 16)
        b = int((data[3:6].hex()), 16)
        c = int((data[6:9].hex()), 16)
        d = int((data[9:12].hex()), 16)
        e = int((data[12:15].hex()), 16)
        # f = int((data[15:18].hex()), 16)
        # print(a)
        # print(b)
        data1 = int(a) / 8388608.0 * 4.0
        data2 = int(b) / 8388608.0 * 4.0
        data3 = int(c) / 8388608.0 * 4.0
        data4 = int(d) / 8388608.0 * 4.0
        dataecg1 = int(e) / 8388608.0 * 4.0
        # print(dataecg1)
        # print(dataecg2)
        ecg = dataecg1
        """
        if ecg < 0:
            ecg = ecg * -1
        """

        LED1.append(data1)
        LED2.append(data2)
        LED3.append(data3)
        LED4.append(data4)
        LISTEECG.append(ecg)

        l1 = np.float32(-math.log10(data1 / LED1[-2]))
        l2 = np.float32(-math.log10(data2 / LED2[-2]))
        l3 = np.float32(-math.log10(data3 / LED3[-2]))
        l4 = np.float32(-math.log10(data4 / LED3[-2]))
        # print("valeur de la liste :" + str(LED1[-2]));
        # print("LED 1 :" +  str(l1))
        # print("LED 1 :" + str(l2))
        # print("LED 1 :" + str(l3))
        # print("LED 1 :" + str(l4))

        temps.append(time.time() - beg)

        """
        t_vec.append(float(temps[-1]))
        y_vals.append(float(data1))

        # calculating heart rate
        t1 = time.time()
        samp_rate = 1 / np.mean(np.diff(t_vec))  # average sample rate for determining peaks
        min_time_bw_samps = (60.0 / heart_rate_span[1])

        # convolve, calculate gradient, and remove bad endpoints
        y_vals = np.convolve(y_vals, np.ones((smoothing_size,)), 'same') / smoothing_size
        red_grad = np.gradient(y_vals, t_vec)
        red_grad[0:int(smoothing_size / 2) + 1] = np.zeros((int(smoothing_size / 2) + 1,))
        red_grad[-int(smoothing_size / 2) - 1:] = np.zeros((int(smoothing_size / 2) + 1,))

        y_vals = np.append(np.repeat(y_vals[int(smoothing_size / 2)], int(smoothing_size / 2)),
                           y_vals[int(smoothing_size / 2):-int(smoothing_size / 2)])
        y_vals = np.append(y_vals, np.repeat(y_vals[-int(smoothing_size / 2)], int(smoothing_size / 2)))

        # peak locator algorithm
        peak_locs = np.where(red_grad < -np.std(red_grad))
        flag = 0
        if len(peak_locs[0]) == 0:
            flag = 1
        if(flag == 0) :
            prev_pk = peak_locs[0][0]
            true_peak_locs, pk_loc_span = [], []
            for ii in peak_locs[0]:
                y_pk = y_vals[ii]
                if (t_vec[ii] - t_vec[prev_pk]) < min_time_bw_samps:
                    pk_loc_span.append(ii)
                else:
                    if pk_loc_span == []:
                        true_peak_locs.append(ii)
                    else:
                        true_peak_locs.append(int(np.mean(pk_loc_span)))
                        pk_loc_span = []

            prev_pk = int(ii)

        t_peaks = [t_vec[kk] for kk in true_peak_locs]
        if t_peaks != [] :
            print('BPM: {0:2.1f}'.format(60.0 / np.mean(np.diff(t_peaks))))
        """
        prev_spo2 = spo2
        spo2, resultat = calc_spo2(l1, l2, l3, l4)
        REC1.append(resultat[0, 0])
        REC2.append(resultat[1, 0])
        REC3.append(resultat[2, 0])
        REC4.append(resultat[3, 0])

        delta_spo2 = np.float32(-math.log10(spo2 / prev_spo2))
        cnt += 1

        '''print("LED1 : " + str(data1))
        print("LED2 : " + str(data2))
        print("LED3 : " + str(data3))
        print("LED4 : " + str(data4))'''
        ''' print("received:", data)
        print(data[0:7])
        a = data[0:8].decode()
        a = ''.join(filter(str.isdigit, a))
        a = int(a)
        if a & (1 << 23):
            a = a | ~((1 << 24) - 1)

        ecg = int(a) / 8388608.0 * 4.0
        print("ECG : " + str(ecg))'''
        # write to file
        # .write(str(time.time()-beg) + ";" + str(LED1) + ";" + str(LED2) + "\n")

        # find threshold (90 % of max value)
        # get sampling frequency (1 / mean of time between two points)
        # apply low pass filter to data
        # Filter requirements.
        if cnt > 1000:
            order = 6
            # fs = 1 / np.mean(np.diff(temps[-100:]))
            fs = 500
            cutoff = 200  # desired cutoff frequency of the filter, Hz
            # print(fs)
            # Get the filter coefficients so we can check its frequency response.
            b, a = butter(order, cutoff / (0.5 * fs), btype='low', analog=False)
            y1 = lfilter(b, a, LED1[-1000:])
            y2 = lfilter(b, a, LED2[-1000:])
            y3 = lfilter(b, a, LED3[-1000:])
            y4 = lfilter(b, a, LED4[-1000:])
            yhat1 = savgol_filter(y1, 51, 3)  # window size 51, polynomial order 3
            yhat2 = savgol_filter(y2, 51, 3)  # window size 51, polynomial order 3
            yhat3 = savgol_filter(y3, 51, 3)  # window size 51, polynomial order 3
            yhat4 = savgol_filter(y4, 51, 3)  # window size 51, polynomial order 3
            if time.time() - beg > chronobpm:
                i = 0
                flag = 0
                # print(str(len(yhat1)))
                while i < len(yhat1) - 1:
                    # print(str(yhat1[i]))
                    if yhat1[i] < yhat1[i + 1]:
                        flag = 1
                        i = i + 1
                    elif yhat1[i] > yhat1[i + 1]:
                        if flag == 1:
                            bpm = bpm + 1
                            i = i + 1
                            flag = 0
                        else:
                            flag = 0
                            i = i + 1
                print("Beats per minute (BPM) : " + str(bpm * 1.1))
                bpm = 0
                dbt = len(LED1)
                chronobpm = chronobpm + 10
            z1 = lfilter(b, a, REC1[-1000:])
            z2 = lfilter(b, a, REC2[-1000:])
            z3 = lfilter(b, a, REC3[-1000:])
            z4 = lfilter(b, a, REC4[-1000:])
            zhat1 = savgol_filter(z1, 51, 3)  # window size 51, polynomial order 3
            zhat2 = savgol_filter(z2, 51, 3)  # window size 51, polynomial order 3
            zhat3 = savgol_filter(z3, 51, 3)  # window size 51, polynomial order 3
            zhat4 = savgol_filter(z4, 51, 3)  # window size 51, polynomial order 3
            """
            if time.time() - beg > chronobpm:
                i = 0
                flag = 0
                print(str(len(zhat1)))
                while i < len(zhat1) - 1:
                    print(str(zhat1[i]))
                    if zhat1[i] < zhat1[i + 1]:
                        flag = 1
                        i = i + 1
                    elif zhat1[i] > zhat1[i + 1]:
                        if flag == 1:
                            bpm = bpm + 1
                            i = i + 1
                            flag = 0
                        else:
                            flag = 0
                            i = i + 1
                print("Nombre de battements par minutes : " + str(bpm * 1.1))
                bpm = 0
                dbt = len(LED1)
                chronobpm = chronobpm + 10
                """
            LISTEECGTEST = LISTEECG[-1000:]
            if time.time() - beg > chronoecg:
                flagecg = 0
                j = 0
                # print(str(len(yhat1)))
                while j < len(LISTEECGTEST) - 1:
                    # print(str(yhat1[i]))
                    if LISTEECGTEST[j] < LISTEECGTEST[j + 1]:
                        flagecg = 1
                        j = j + 1
                    elif LISTEECGTEST[j] > LISTEECGTEST[j + 1]:
                        if flagecg == 1:
                            bpm_ecg = bpm_ecg + 1
                            j = j + 1
                            flagecg = 0
                        else:
                            flagecg = 0
                            j = j + 1
                    else:
                        j = j + 1
                print("Beats per minute for ECG : " + str(bpm_ecg * 1.1 / 2))
                bpm_ecg = 0
                chronoecg = chronoecg + 10

            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(temps[-1]), str(yhat1[-1]), str(yhat2[-1]), str(yhat3[-1]), str(yhat4[-1]),
                                 str(float(zhat1[-1])), str(float(zhat2[-1])), str(float(zhat3[-1])),
                                 str(float(zhat4[-1])), str(spo2), str(delta_spo2), str(ecg)])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()
        else:
            with open("data.csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [str(temps[-1]), str(data1), str(data2), str(data3), str(data4), str(float(resultat[0, 0])),
                     str(float(resultat[1, 0])), str(float(resultat[2, 0])), str(float(resultat[3, 0])), str(spo2),
                     str(delta_spo2), str(ecg)])
                # writer.writerow([str(time.time()-beg), str(ecg)])
                file.flush()

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        print("Connected, start typing and press ENTER...")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

        while True:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

            # data will be empty on EOF (e.g. CTRL+D on *nix)
            if not data:
                break

            # some devices, like devices running MicroPython, expect Windows
            # line endings (uncomment line below if needed)
            # data = data.replace(b"\n", b"\r\n")

            # Writing without response requires that the data can fit in a
            # single BLE packet. We can use the max_write_without_response_size
            # property to split the data into chunks that will fit.

            for s in sliced(data, rx_char.max_write_without_response_size):
                await client.write_gatt_char(rx_char, s)

            print("sent:", data)


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass
