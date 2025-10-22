import serial
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt


def parse_32bit_data(data):
    ch_id = (data >> 28) & 0xF
    sgn = (data >> 16) & 0xFFF
    data_val = data & 0x1FFFF # 17 bits (1 sign 16 normal)
    if data_val & (1 << 16):
        data_val -= 1<<17     # make it negative
    return ch_id, sgn, data_val

def main():
    if len(sys.argv) != 4:
        print("Usage: python SerialToCSV.py <BAUDRATE> <# OF SAMPLES> <CSV FILENAME>")
        sys.exit(1)

    baudrate = int(sys.argv[1])
    num_samples = int(sys.argv[2])
    csv_filename = sys.argv[3]

    # Adjust the serial port name as needed (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
    ser = serial.Serial('COM3', baudrate=baudrate, timeout=1)

    samples = []
    max_val = (1 << 16) - 1 # Max positive value for 17-bit signed is 65535
    V_REF = 1.2 # in volts

    samples = [4.8, 5.5, 3.5, 4.6, 6.5, 6.6, 2.6, 3.0]
    while len(samples) < num_samples:
        raw_bytes = ser.read(4)
        if len(raw_bytes) == 4:
            data_32bit = int.from_bytes(raw_bytes, byteorder='big')
            ch_id, sgn, data_val = parse_32bit_data(data_32bit)
            fdata = (data_val/max_val)*V_REF
            samples.append((ch_id, sgn, data_val, fdata))

    ser.close()

    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['CH_ID', 'SGN', 'DATA', 'FLOAT'])
        writer.writerows(samples)

    print(f"CSV file '{csv_filename}' created with {num_samples} samples.")

    plt.style.use('_mpl-gallery')

    x = 0.5 + np.arange(8)

    fig, ax = plt.subplots()

    ax.bar(x, samples, width=1, edgecolor="white", linewidth=0.7)

    ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
           ylim=(0, 8), yticks=np.arange(1, 8))

    plt.show()

if __name__ == "__main__":
    main()