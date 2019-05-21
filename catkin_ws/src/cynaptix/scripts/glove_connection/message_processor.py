import rospy

import struct

from cynaptix.msg import GloveMeasuredData
from cynaptix.msg import GloveTarget

# Store the target servo positions and LRA magnitudes
servos = [0.0, 0.0, 0.0]
vibrations = [0, 0, 0,
              0, 0, 0,
              0, 0, 0]

# Store the most recently received servo positions and torques
servo_recs = [0.0, 0.0, 0.0]
              
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

pub = None
sub = None


def publish_data():
    global servo_recs
    global servo_torques
    global pub

    # Publish the servo positions and torques
    data = GloveMeasuredData()

    data.thumb_pos = servo_recs[0]
    data.index_pos = servo_recs[1]
    data.middle_pos = servo_recs[2]

    pub.publish(data)


def receive_data_callback(data):
    global servos
    global vibrations

    servos[0] = data.thumb_target
    servos[1] = data.index_target
    servos[2] = data.middle_target

    vibrations[0] = data.thumb_vib_1
    vibrations[1] = data.thumb_vib_2
    vibrations[2] = data.thumb_vib_3

    vibrations[3] = data.index_vib_1
    vibrations[4] = data.index_vib_2
    vibrations[5] = data.index_vib_3

    vibrations[6] = data.middle_vib_1
    vibrations[7] = data.middle_vib_2
    vibrations[8] = data.middle_vib_3


def ros_worker():
    global pub
    global sub

    pub = rospy.Publisher('glove_measured_data', 
                          GloveMeasuredData, 
                          queue_size=1)
    sub = rospy.Subscriber('glove_target', 
                           GloveTarget, 
                           receive_data_callback)

    # Handle message callbacks
    rospy.spin()


def float_to_bytes(val):
    # mbed is little endian, so force little endiannes
    return struct.pack('<f', val)


def bytes_to_float(val):
    # As above
    return struct.unpack('<f', val)[0]


def get_glove_message():
    global vibrations
    global servos

    # Start the message
    msg = bytearray([MESSAGE_START])

    # Add all the servo motor values
    msg += bytearray([TARGET_MOTOR_POS, THUMB])
    msg += bytearray(float_to_bytes(servos[0]))
    msg += bytearray([TARGET_MOTOR_POS, INDEX])
    msg += bytearray(float_to_bytes(servos[1]))
    msg += bytearray([TARGET_MOTOR_POS, MIDDLE])
    msg += bytearray(float_to_bytes(servos[2]))

    # Add all the vibration motor values
    msg += bytearray([VIBRATIONS])
    msg += bytearray(vibrations)

    # End of message
    msg += bytearray([MESSAGE_END])

    return msg


def process_glove_data(data):
    global reading_message
    global servo_torques
    global servo_recs

    b = bytearray()
    b.extend(data)

    iterator = b.__iter__()
    for c in iterator:
        if c == MESSAGE_START:
            reading_message = True
        elif reading_message:
            if c == MESSAGE_END:
                reading_message = False

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

                # Store in the correct spot
                if motor == THUMB:
                    servo_recs[0] = fnum
                elif motor == INDEX:
                    servo_recs[1] = fnum
                elif motor == MIDDLE:
                    servo_recs[2] = fnum
                else:
                    rospy.logerr("Received invalid position motor specifier.")

            else:
                rospy.logerr("Received invalid command while reading.")

        else:
            rospy.logerr("Received command before reading.")

    publish_data()
