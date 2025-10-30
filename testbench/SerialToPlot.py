import serial
import csv
import sys
from collections import Counter
import matplotlib.pyplot as plt

def main():
    if len(sys.argv) != 4:
        print("Usage: python SerialToCSV_Hex.py <PORT> <BAUDRATE> <# OF SAMPLES>")
        sys.exit(1)

    port = sys.argv[1]
    baudrate = int(sys.argv[2])
    num_samples = int(sys.argv[3])
    csv_filename = "serial_data.csv"

    ser = serial.Serial(port, baudrate=baudrate, timeout=1)

    samples = []
    sample_count = 0

    print(f"Reading {num_samples} samples from {port} at {baudrate} baud...")

    while sample_count < num_samples:
        raw_bytes = ser.read(4)
        if len(raw_bytes) == 4:
            data_32bit = int.from_bytes(raw_bytes, byteorder='big')

            # Extract least-significant 17 bits
            data_17bit = data_32bit & 0x1FFFF

            hex_str = f"0x{data_32bit:08X}"
            bin_str = f"{data_17bit:017b}"  # 17-bit binary string

            samples.append((sample_count, hex_str, bin_str))
            sample_count += 1

    ser.close()

    # Write to CSV
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['SAMPLE_NUM', 'HEX_VALUE', 'BINARY_17BIT'])
        writer.writerows(samples)

    print(f"CSV file '{csv_filename}' created with {num_samples} samples.")

    # --- Histogram of occurrences based on 17-bit values ---
    values = [s[2] for s in samples]  # binary 17-bit strings
    counts = Counter(values)

    plt.figure(figsize=(10, 5))
    plt.bar(counts.keys(), counts.values(), width=0.6)
    plt.title("Occurrences of LSB 17-bit Patterns")
    plt.xlabel("Binary 17-bit Value")
    plt.ylabel("Occurrences")
    plt.xticks(rotation=90)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
