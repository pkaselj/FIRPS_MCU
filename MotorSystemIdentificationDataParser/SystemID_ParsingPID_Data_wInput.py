from enum import Enum
import math
import os
import csv
from typing import List, Tuple
from matplotlib import pyplot as plt
import struct


# ----------------------------------------------------------------- #
#                            SETTINGS                               #
# ----------------------------------------------------------------- #

INPUT_FILE = r'MotorSamples\PulsePosition\pid\setpoint_30_sampling_f_62_5_006_live.bin'
# INPUT_FILE = r'C:\Users\KASO\Desktop\MotorSamples\100hz_30.txt'

sampling_time_interval = 1/62.5


# ----------------------------------------------------------------- #
#                                                                   #
# ----------------------------------------------------------------- #

def parse_line(line : bytes) -> Tuple[int, float, float, float]:
    try:
        sample_nr = int.from_bytes(line[0:2], 'little')
        [setpoint] = struct.unpack('f', line[2:6])
        [rpm] = struct.unpack('f', line[6:10])
        [input] = struct.unpack('f', line[10:14])
        return (sample_nr, setpoint, rpm, input)
    except:
        return None

def parse_file(file_path) -> List[Tuple[int, float, float, float]]:
    with open(file_path, 'rb') as f:
        data = f.read()
        N = 16
        data = [data[i:i + N] for i in range(0, len(data), N)]
        data = [parse_line(x) for x in data]
        data = [x for x in data if x is not None]

        i_data = [i for (i, r, x, u) in data]
        r_data = [r for (i, r, x, u) in data]
        x_data = [x for (i, r, x, u) in data]
        u_data = [u for (i, r, x, u) in data]

        return (i_data, r_data, x_data, u_data)
        

def plot_data(data : Tuple[int, int, int]):
    (i, r, x, u) = data

    i = [v * sampling_time_interval for v in i]

    plt.plot(i, r, color='b', label='r(t)')
    plt.plot(i, x, color='r', label='x(t)')
    plt.plot(i, u, color='g', label='u(t)')

    plt.legend()
    plt.show()


if __name__ == '__main__':

    file_path = INPUT_FILE
    data = parse_file(file_path)

    for row in data:
        print(f'{repr(row)}\n')

    plot_data(data)