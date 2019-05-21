#!/usr/bin/env python

import socket
import struct

pos = [0.0, 0.0, 0.0]
vibrations = [0, 0, 0,
              0, 0, 0,
              0, 0, 0]

# Message information bytes
MESSAGE_START = 0xFF
MESSAGE_END = 0xFE

TARGET_MOTOR_POS = 0x03
ACTUAL_MOTOR_POS = 0x05
VIBRATIONS = 0x06

THUMB = 0x06
INDEX = 0x07
MIDDLE = 0x08

reading_message = False


def float_to_bytes(val):
    # mbed is little endian, so force little endiannes
    return struct.pack('<f', val)


def bytes_to_float(val):
    # As above
    return struct.unpack('<f', val)[0]


def get_reply_msg():
    global pos

    # Start the message
    msg = bytearray([MESSAGE_START])

    # Add all the motor positions
    msg += bytearray([ACTUAL_MOTOR_POS, THUMB])
    msg += bytearray(float_to_bytes(pos[0]))
    msg += bytearray([ACTUAL_MOTOR_POS, INDEX])
    msg += bytearray(float_to_bytes(pos[1]))
    msg += bytearray([ACTUAL_MOTOR_POS, MIDDLE])
    msg += bytearray(float_to_bytes(pos[2]))

    # End of message
    msg += bytearray([MESSAGE_END])

    print("Got reply message\n")

    return msg


def process_data(data):
    global pos
    global reading_message

    b = bytearray()
    b.extend(data)

    iterator = b.__iter__()
    for c in iterator:
        if c == MESSAGE_START:
            reading_message = True
        elif reading_message:
            if c == MESSAGE_END:
                reading_message = False

            elif c == VIBRATIONS:
                for i in range(9):
                    val = iterator.next()
                    vibrations[i] = val

            elif c == TARGET_MOTOR_POS:
                # Check which motor it is
                motor = iterator.next()

                # Read the 4 byte float
                num = bytearray([iterator.next(),
                                 iterator.next(),
                                 iterator.next(),
                                 iterator.next()])

                # Convert to float
                fnum = bytes_to_float(num)

                # Store the value in the correct spot
                if motor == THUMB:
                    pos[0] = fnum
                elif motor == INDEX:
                    pos[1] = fnum
                elif motor == MIDDLE:
                    pos[2] = fnum
                else:
                    print("Invalid pos message!\n")
            else:
                print("Invalid message type!\n")
        else:
            print("Message arrived before message start!\n")

    print("Data processed!\n")


if __name__ == '__main__':
    port = 25565
    host = "nickick-laptop"
    timeout = 0.3

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))

    while(True):
        data = s.recv(1024)
        process_data(data)
        print("pos:")
        print(pos)
        print("\nvibrations:")
        print(vibrations)
        print("\n\n")
        msg = get_reply_msg()
        s.send(msg)
