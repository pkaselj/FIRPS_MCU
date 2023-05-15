from enum import Enum
import math
import os
import csv
from typing import List, Tuple
from matplotlib import pyplot as plt
 

# RPS = revolutions per second
# PPR = pulses per rotation

# ----------------------------------------------------------------- #
#                            SETTINGS                               #
# ----------------------------------------------------------------- #

INPUT_FILE = r'MotorSamples\PulsePosition\day2\30_001_m3.bin'

F_CPU = 16e6

PULSES_PER_ROTATION = 7*35

# ----------------------------------------------------------------- #
#                                                                   #
# ----------------------------------------------------------------- #


# Takes a line represented by a byte array and returns
# a tuple of parsed data
def parse_line(line : bytes) -> Tuple[int, int]:
    sample_nr = int.from_bytes(line[0:2], 'little')
    Tp = int.from_bytes(line[2:6], 'little')
    return (sample_nr, Tp)

# Takes a file path and returns tuple of parsed values
def parse_file(file_path) -> List[Tuple[int, int]]:
    with open(file_path, 'rb') as f:
        data = f.read()
        N = 7
        data = [data[i:i + N] for i in range(0, len(data), N)]
        data = [parse_line(x) for x in data]

        i_data = [i for (i, Tp) in data]
        Tp_data = [Tp for (i, Tp) in data]

        return (i_data, Tp_data)

def get_ideal_reponse(t : Tuple[int], tau : int, K : int) -> Tuple[float]:
    y = [K * (1 - math.exp(-x/tau)) for x in t]
    return y

def plot_data(data : Tuple[int, int, int]) -> Tuple[float, float]:
    (i, Tp) = data

    spd = get_speed(Tp)

    avg_spd_value = sum(spd)/len(spd)
    avg_spd = [avg_spd_value] * len(i)
    
    tau_p_speed_value = 0.632 * avg_spd_value
    tau_p_speed = [tau_p_speed_value] * len(i)

    Kp = avg_spd_value

    tau_p_index = next(i for i,v in enumerate(spd) if v >= tau_p_speed_value)
    tau_p = sum(Tp[:tau_p_index]) * 1/F_CPU

    t = [sum(Tp[:x]) * 1/F_CPU for x in i]

    y = get_ideal_reponse(t, tau_p, Kp)


    plt.plot(i, spd, color='r', label='Measured Response')
    plt.plot(i, avg_spd, color='k', marker='_', label='Steady State')
    plt.plot(i, tau_p_speed, color='g', marker='_', label='0.632 * Steady State')
    plt.plot(i, y, color='b', label='FOPDT Model')
    plt.legend()
    plt.show()

    return tau_p_index, tau_p, spd, Kp


def get_speed(Tp : Tuple[int]) -> Tuple[int]:
    speed = [0] * len(Tp)

    for i, current_t in enumerate(Tp):
        if current_t != 0:
            speed[i] = F_CPU/(current_t * PULSES_PER_ROTATION)
    # speed = [x for x in Tp] # DEBUG
    return speed


def get_new_file_path(old_file_path) -> str:
    dir, file_name = os.path.split(old_file_path)
    file_name_without_ext, _ = os.path.splitext(file_name)
    new_file_name = file_name_without_ext + '.csv'
    new_file_path = os.path.join(new_file_name, dir)
    return new_file_path

if __name__ == '__main__':

    file_path = INPUT_FILE
    data = parse_file(file_path)
    new_file_path = get_new_file_path(file_path)

    print(f'{new_file_path}\n')
    for row in data:
        print(f'{repr(row)}\n')

    tau_p_i, tau_p, spd, Kp = plot_data(data)

    print(f'Tau_p index = {tau_p_i}\nTau_p = {tau_p} s\nKp * delta u = {Kp}')