import pykondo

class KondoICS(object):
    def __init__(self, port, baudrate, readback_echo=False):
        """ Constructor takes serial port and baudrate as arguments. """

        self.ics = ICSData()
        ret = ics_init(ics)
        if ret < 0:
            sys.exit(ics.error)

    def send(self, data):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write(data)
        if self.readback_echo:
            self.ser.read(len(data))

    def recv(self, size):
        try:
            self.ser.read(size)
        except:
            raise Exception('recv')
        
    def set_position(self, id, position):
        with self.serial_mutex:
            self.write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)#0.00235)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data
    
    def write(self, servo_id, address, data):
    
        """ Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module dynamixel_const
        for readability. "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION_L, (20, 1))
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 3 + len(data)  # instruction, address, len(data), checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
        packet.extend(data)
        packet.append(checksum)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data
