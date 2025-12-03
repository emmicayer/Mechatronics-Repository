from pyb import I2C, delay
import struct

class BNO055:
    # Class attributes 
    # I2C addresses
    addr_high = 0x29        # COM is high (Vcc), 0x29 is address 
    addr_low = 0x28         # COM is low (ground), 0x28 is address  
    
    # Registers - page 56 in data sheet, identifies registers (memory locations) inside IMU's chip. each holds data about yaw, gyro, mode, etc
    reg_chip_id = 0x00      # Written as 0
    reg_page_id = 0x07      # Written as 7
    reg_eul_heading = 0x1A  # LSB of data, since BNO055 uses little endian (LSB first 1A, MSB 1B)
    reg_eul_roll = 0x1C
    reg_eul_pitch = 0x1E
    reg_gyr_x = 0x14
    reg_gyr_y = 0x16
    reg_gyr_z = 0x18
    reg_calib_stat = 0x35
    reg_opr_mode = 0x3D
    reg_sys_trigger = 0x3F
    reg_calib_start = 0x55  # Calibration offset, 22 bytes of calibration data. 55-5F acc, 60-6a mag, 6B-71 gyr. X, Y, Z
    calib_len = 22          # Lengh of calibration
    
    # Operation modes (fusion mode)
    mode_config = 0x00
    mode_imu = 0x08
    mode_compass = 0x09
    mode_mag = 0x0A
    mode_ndof_fmc_off = 0x0B
    mode_ndof = 0x0C

    # Scaling
    _deg2rad = 0.01745329251    # Convert degrees to radians
    _eul_scale = _deg2rad/16.0 
    _gyr_scale = _deg2rad/16.0

    # Initialization and methods
    def __init__(self, i2c, addr=addr_low):
        self.i2c = i2c
        self.addr = addr    # IMU's i2c address 

        # Probe chip_id to see if IMU is correctly connected
        try:
            cid = int.from_bytes(self.i2c.mem_read(1, self.addr, self.reg_chip_id), "little")
        except OSError:
            raise OSError("BNO055 not responding at 0x{:02X}".format(addr))

        if cid != 0xA0:
            delay(700)
            cid = int.from_bytes(self.i2c.mem_read(1, self.addr, self.reg_chip_id), "little")
            if cid != 0xA0:
                raise OSError("Unexpected CHIP_ID 0x{:02X} (expect 0xA0)".format(cid))

        self.change_operating_mode(self.mode_config)    # Put IMU into correct mode
        delay(20)                                   
        self.i2c.mem_write(bytes([0x00]), self.addr, self.reg_page_id)  # Make sure IMU reguster page is set to 0, which has main sensor data
                                                                        # bytes = single byte with value 0. Write 0x00 to register 0x07 on device 0x28
                                            # Turns a number into a 1 byte integer that the IMU understands
    def change_operating_mode(self, mode):  # Calls current BNO055 object so it can access self.i2c and self.addr. Sets BNO055 operating mode
        data_bytes = bytes([mode & 0xFF])   # Creates 1 byte binary object that can be sent to i2c bus from the integer mode code (keeps lowest 8 bits)
        self.i2c.mem_write(data_bytes, self.addr, self.reg_opr_mode)    # reg_opr_mode = reg address. Write this mode num into opp_mode register of IMU
        delay(20)                           # Datasheet says we need to wait 19+ ms before sensor is stable. lets IMU settle 

    def retrieve_cal_status(self):          # Return raw calibration status byte, 7-6 is system, 5-4 is gyro, 3-2 is acc, 1-0 is mag. 
        return int.from_bytes(self.i2c.mem_read(1, self.addr, self.reg_calib_stat), "little") # 0 is uncalibrated, 3 is fully calibrated. 
                                                                                    # reads 1 byte from register address stored in self.reg_cal_stat
    
    def read_cal_coeff(self):               # Read calibration coeff (offsets the chip learned during calibration)
        self.change_operating_mode(self.mode_config)    # Call in config mode
        delay(20)                           
        return bytes(self.i2c.mem_read(self.calib_len, self.addr, self.reg_calib_start)) # Reads 22 bytes starting at start (0x55). bytes() ensures immutable
    
    def write_cal_coeff(self, imu_data):
        if not isinstance(imu_data, (bytes, bytearray)) or len(imu_data) != self.calib_len: # Checks type of imu_data, isinstance(x, T) returns True if x is type T
            raise ValueError("Calibration imu_data must be 22 bytes")
        self.change_operating_mode(self.mode_config)
        delay(20)                           # Lets IMU settle before writing
        self.i2c.mem_write(imu_data, self.addr, self.reg_calib_start)
        delay(20)

    def read_euler_angles(self):                                    # b is small bytes object 
        b = self.i2c.mem_read(2, self.addr, self.reg_eul_heading)   # Read 2 bytles starting at LSB. Each angle 16 bit signed integer (little endian) (deg/16)
        h = struct.unpack("<h", b)[0]*self._eul_scale               # Converts 2 bites into signed 16-bit int (< little endian, h = signed short)
        b = self.i2c.mem_read(2, self.addr, self.reg_eul_roll)      # Same for roll
        r = struct.unpack("<h", b)[0]*self._eul_scale
        b = self.i2c.mem_read(2, self.addr, self.reg_eul_pitch)     # Same for pitch
        p = struct.unpack("<h", b)[0]*self._eul_scale
        return(h, r, p)                                             # returns tuple in radians
    
    def read_angular_velocity(self):                                # Gyro angular rates x, y, z in rad/s
        b = self.i2c.mem_read(2, self.addr, self.reg_gyr_x)         # Read 2 bytes for x gyro from IMU via i2c
        gx = struct.unpack("<h", b)[0]*self._gyr_scale              # Struct converts raw bytes to numbers, little endian signed short int, [0] takes first element
        b = self.i2c.mem_read(2, self.addr, self.reg_gyr_y)         # Same for y
        gy = struct.unpack("<h", b)[0]*self._gyr_scale
        b = self.i2c.mem_read(2, self.addr, self.reg_gyr_z)         # Same for z
        gz = struct.unpack("<h", b)[0]*self._gyr_scale
        return (gx, gy, gz)