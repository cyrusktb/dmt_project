import rospy

import struct

from cynaptix.msg import FrameMeasuredData
from cynaptix.msg import FrameTarget

# Store the target motor positions
target_pos = [0.0, 0.0, 0.0, 0.0, 0.0]

# Store the most recently received motor positions and torques
motor_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
motor_torques = [0.0, 0.0, 0.0, 0.0, 0.0]

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

pub = None
sub = None

def publish_data():
    global motor_pos
    global motor_torques
    global pub

    # Publish the servo positions and torques
    data = FrameMeasuredData()

    data.x_pos = motor_pos[0]
    data.y_pos = motor_pos[1]
    data.z_pos = motor_pos[2]
    data.theta_pos = motor_pos[3]
    data.grabber_pos = motor_pos[4]

    data.x_torque = motor_torques[0]
    data.y_torque = motor_torques[1]
    data.z_torque = motor_torques[2]
    data.theta_torque = motor_torques[3]
    data.grabber_torque = motor_torques[4]

    pub.publish(data)


def receive_data_callback(data):
    global target_pos

    target_pos[0] = data.x_pos
    target_pos[1] = data.y_pos
    target_pos[2] = data.z_pos
    target_pos[3] = data.theta_pos
    target_pos[4] = data.grabber_pos


def ros_worker():
    global pub
    global sub

    pub = rospy.Publisher('frame_measured_data',
                          FrameMeasuredData,
                          queue_size=1)
    sub = rospy.Subscriber('frame_target',
                           FrameTarget,
                           receive_data_callback)

    # Handle message callbacks
    rospy.spin()


def float_to_bytes(val):
    # mbed is little endian, so force little endiannes
    return struct.pack('<f', val)


def bytes_to_float(val):
    # As above
    return struct.unpack('<f', val)[0]


def get_frame_message():
    global target_pos

    # Start the message
    msg = bytearray([MESSAGE_START])

    # Add all the motor target values
    msg += bytearray([TARGET_MOTOR_POS, X_AXIS])
    msg += bytearray(float_to_bytes(target_pos[0]))
    msg += bytearray([TARGET_MOTOR_POS, Y_AXIS])
    msg += bytearray(float_to_bytes(target_pos[1]))
    msg += bytearray([TARGET_MOTOR_POS, Z_AXIS])
    msg += bytearray(float_to_bytes(target_pos[2]))
    msg += bytearray([TARGET_MOTOR_POS, THETA])
    msg += bytearray(float_to_bytes(target_pos[3]))
    msg += bytearray([TARGET_MOTOR_POS, GRABBER])
    msg += bytearray(float_to_bytes(target_pos[4]))

    # End of message
    msg += bytearray([MESSAGE_END])

    return msg


def process_frame_data(data):
    global reading_message
    global motor_pos
    global motor_torques

    b = bytearray()
    b.extend(data)

    iterator = b.__iter__()
    for c in iterator:
        if c == MESSAGE_START:
            reading_message = True
        elif reading_message:
            if c == MESSAGE_END:
                reading_message = False
            elif c == MOTOR_TORQUE:
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
                    motor_torques[0] = fnum
                elif motor == Y_AXIS:
                    motor_torques[1] = fnum
                elif motor == Z_AXIS:
                    motor_torques[2] = fnum
                elif motor == THETA:
                    motor_torques[3] = fnum
                elif motor == GRABBER:
                    motor_torques[4] = fnum
                else:
                    rospy.logerr("Received invalid torque motor specifier.")
            
            elif c == ACTUAL_MOTOR_POS:
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
                    motor_pos[0] = fnum
                elif motor == Y_AXIS:
                    motor_pos[1] = fnum
                elif motor == Z_AXIS:
                    motor_pos[2] = fnum
                elif motor == THETA:
                    motor_pos[3] = fnum
                elif motor == GRABBER:
                    motor_pos[4] = fnum
                else:
                    rospy.logerr("Received invalid position motor specifier.")
            
            else:
                rospy.logerr("Received invalid command while reading.")

        else:
            rospy.logerr("Received command before reading.")

    publish_data()
