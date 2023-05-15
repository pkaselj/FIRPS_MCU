from enum import Enum
import math
import os
import csv
from typing import List, Tuple
from matplotlib import pyplot as plt
 
class Mode(Enum):
    MULTI_PULSE_PER_SAMPLE = 0
    MULTI_SAMPLE_PER_PULSE = 1

# RPS = revolutions per second
# PPR = pulses per rotation

# ----------------------------------------------------------------- #
#                            SETTINGS                               #
# ----------------------------------------------------------------- #

INPUT_FILE = r'MotorSamples\1khz_50_001.bin'
# INPUT_FILE = r'MotorSamples\100hz_30.txt'

# sampling_time_interval = (260.1/2) * 1e-6 # 8kHz
sampling_time_interval = (2.02/2) * 1e-3 # 1kHz
# sampling_time_interval = (20.03/2) * 1e-3 # 100Hz

# PULSES_PER_ROTATION = 7 * 35
# PULSES_PER_ROTATION = 35
# PULSES_PER_ROTATION = 7
PULSES_PER_ROTATION = 1

# RPS_MODE = Mode.MULTI_SAMPLE_PER_PULSE
RPS_MODE = Mode.MULTI_PULSE_PER_SAMPLE

# ----------------------------------------------------------------- #
#                                                                   #
# ----------------------------------------------------------------- #

# RPS_calculator = lambda N, Ts, PPR : RPS_calculator_multiple_pulses_per_sample(N, Ts, PPR)
RPS_calculator = lambda N, Ts, PPR : RPS_calculator_multiple_samples_per_pulse(N, Ts, PPR)

def parse_line(line : bytes) -> Tuple[int, int, int]:
    try:
        sample_nr = int.from_bytes(line[0:4], 'little')
        input_value = int(line[5])
        response_value = int.from_bytes(line[7:11], 'little')
        return (sample_nr, input_value, response_value)
    except:
        return None

def parse_file(file_path) -> List[Tuple[int, int, int]]:
    with open(file_path, 'rb') as f:
        data = f.read()
        N = 12
        data = [data[i:i + N] for i in range(0, len(data), N)]
        data = [parse_line(x) for x in data]
        data = [x for x in data if x is not None]

        i_data = [i for (i, u, x) in data]
        u_data = [u for (i, u, x) in data]
        x_data = [x for (i, u, x) in data]

        return (i_data, u_data, x_data)
        
def RPS_calculator_multiple_pulses_per_sample(N, Ts, PPR) -> int:
    return N / (Ts * PPR)

def RPS_calculator_multiple_samples_per_pulse(N, Ts, PPR) -> int:
    return 1 / (N * Ts * PPR)

def plot_data(data : Tuple[int, int, int]) -> Tuple[float, float]:
    (i, u, x) = data

    spd = get_speed(x)

    avg_spd_value = sum(spd)/len(spd)
    avg_spd = [avg_spd_value] * len(i)

    duty_cycle = u[0]
    Kp = avg_spd_value / duty_cycle
    
    tau_p_speed_value = 0.632 * avg_spd_value
    tau_p_speed = [tau_p_speed_value] * len(i)

    tau_p_index = next(i for i,v in enumerate(spd) if v >= tau_p_speed_value)

    plt.plot(i, u, color='b', label='u(t)')
    plt.plot(i, spd, color='r', label='x(t)')
    plt.plot(i, avg_spd, color='k', marker='_', label='Steady State')
    plt.plot(i, tau_p_speed, color='g', marker='_', label='0.632 * Steady State')
    plt.legend()
    plt.show()

    return (Kp, tau_p_index)



def get_speed(data : Tuple[int]) -> Tuple[int]:
    spikes = get_derivative(data)

    speed = [0] * len(spikes)

    previous_spike_index = 0
    current_speed = 0

    for i in range(1, len(spikes)):
        if spikes[i] > 0: # is a spike
            Ts = sampling_time_interval
            PPR = PULSES_PER_ROTATION
            if RPS_MODE == Mode.MULTI_SAMPLE_PER_PULSE:
                N = i - previous_spike_index    
                current_speed = RPS_calculator_multiple_samples_per_pulse(N, Ts, PPR)
            else:
                N = spikes[i]
                current_speed = RPS_calculator_multiple_pulses_per_sample(N, Ts, PPR)
            previous_spike_index = i

        speed[i] = current_speed

    return speed


def get_derivative(data : Tuple[int]) -> Tuple[int]:
    result = [data[i] - data[i-1] for i in range(1, len(data))]
    result = [0] + result # initial condition
    return result

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

    Kp, tau_p_i = plot_data(data)
    tau_p = tau_p_i * sampling_time_interval

    print(f'Kp = {Kp}\nTau_p index = {tau_p_i}\nTau_p = {tau_p} s')