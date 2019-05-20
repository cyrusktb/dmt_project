#!/usr/bin/env python

import socket
import struct

pos = [0.0, 0.0, 0.0, 0.0, 0.0]
torques = [3.0, 100000, -1.0, 0.255361, -0.0002179]

# Message information bytes
MESSAGE_START = 0xFF
MESSAGE_END = 0xFE

TARGET_MOTOR_POS = 0x03
MOTOR_TORQUE = 0x04
ACTUAL_MOTOR_POS = 0x05

X_AXIS = 0x01
Y_AXIS = 0x02
Z_AXIS = 0x03
THETA = 0x04
GRABBER = 0x05

reading_message = False


def float_to_bytes(val):
    # mbed is little endian, so force little endiannes
    return struct.pack('<f', val)


def bytes_to_float(val):
    # As above
    return struct.unpack('<f', val)[0]


def get_reply_msg():
    global pos
    global torques

    # Start the message
    msg = bytearray([MESSAGE_START])

    # Add all the motor positions
    msg += bytearray([ACTUAL_MOTOR_POS, X_AXIS])
    msg += bytearray(float_to_bytes(pos[0]))
    msg += bytearray([ACTUAL_MOTOR_POS, Y_AXIS])
    msg += bytearray(float_to_bytes(pos[1]))
    msg += bytearray([ACTUAL_MOTOR_POS, Z_AXIS])
    msg += bytearray(float_to_bytes(pos[2]))
    msg += bytearray([ACTUAL_MOTOR_POS, THETA])
    msg += bytearray(float_to_bytes(pos[3]))
    msg += bytearray([ACTUAL_MOTOR_POS, GRABBER])
    msg += bytearray(float_to_bytes(pos[4]))

    # Add all the motor torques
    msg += bytearray([MOTOR_TORQUE, X_AXIS])
    msg += bytearray(float_to_bytes(torques[0]))
    msg += bytearray([MOTOR_TORQUE, Y_AXIS])
    msg += bytearray(float_to_bytes(torques[1]))
    msg += bytearray([MOTOR_TORQUE, Z_AXIS])
    msg += bytearray(float_to_bytes(torques[2]))
    msg += bytearray([MOTOR_TORQUE, THETA])
    msg += bytearray(float_to_bytes(torques[3]))
    msg += bytearray([MOTOR_TORQUE, GRABBER])
    msg += bytearray(float_to_bytes(torques[4]))

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
                if motor == X_AXIS:
                    pos[0] = fnum
                elif motor == Y_AXIS:
                    pos[1] = fnum
                elif motor == Z_AXIS:
                    pos[2] = fnum
                elif motor == THETA:
                    pos[3] = fnum
                elif motor == GRABBER:
                    pos[4] = fnum
                else:
                    print("Invalid pos message!\n")
            else:
                print("Invalid message type!\n")
        else:
            print("Message arrived before message start!\n")

    print("Data processed!\n")


if __name__ == '__main__':
    port = 25566
    host = "nickick-laptop"
    timeout = 0.3

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))

    while(True):
        data = s.recv(1024)
        process_data(data)
        print("pos:")
        print(pos)
        print("\ntorques:")
        print(torques)
        print("\n\n")
        msg = get_reply_msg()
        s.send(msg)
