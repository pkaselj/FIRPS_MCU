import serial
import time
import struct
import msvcrt
import matplotlib.pyplot as plt

def decode_uint8(data : bytes) -> int:
    return int.from_bytes(data, 'little')

def decode_float4(data : bytes) -> int:
    return struct.unpack('<f', data)[0]

def decode_uint32(data : bytes) -> int:
    return int.from_bytes(data, 'little')

if __name__ == '__main__':

    serial_port = serial.Serial()
    serial_port.baudrate = 115200
    serial_port.parity = serial.PARITY_NONE
    serial_port.port = 'COM21'
    serial_port.bytesize = serial.EIGHTBITS
    serial_port.stopbits = serial.STOPBITS_ONE

    print(f'Opening Serial port ({serial_port.baudrate}/{repr(serial_port.bytesize)}/{repr(serial_port.stopbits)}/{repr(serial_port.parity)}/{serial_port.port})...')

    serial_port.open()

    print(f'Serial port {serial_port.port} opened!')

    time.sleep(3)

    buffer = []

    while True:
        # If user pressed a key to send
        if msvcrt.kbhit():
            pressed_key = msvcrt.getch()
            pressed_key = pressed_key[0]

            # print(pressed_key)

            # Special key to toggle serial port
            if pressed_key == 27: # ESC
                if serial_port.isOpen():
                    serial_port.close()
                    print('Closed serial port.')
                else:
                    serial_port.open()
                    print('Opened serial port.')
                buffer = []
            # Plot data
            elif pressed_key == 112: # letter 'p'
                plt.plot(buffer)
                plt.show()
                buffer = []
            # Normal key press just to relay to the port
            else:
                if serial_port.isOpen():
                    serial_port.write(pressed_key.to_bytes(1,'little'))
                else:
                    print('Cannot write to a closed port.')
        
        if serial_port.isOpen() and serial_port.in_waiting > 0:
            # Read first byte of the frame (must be STX)
            data = serial_port.read(1)

            # Discard all frames not beginning with 0x02 (STX)
            if data != 0x02:
                pass

            # Read rest of the data
            if serial_port.in_waiting >= 4:
                data = serial_port.read(4)
                # data = data[::-1]
                number = decode_float4(data)
                print(f'0x{data.hex().upper()} :: {number}')
                buffer.append(number)
        
        time.sleep(0.1)

    serial_port.close()
        