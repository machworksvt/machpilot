import gpiod
import time
import threading
from logzero import logger

# from https://developer.nvidia.com/embedded/learn/jetson-nano-2gb-devkit-user-guide
DEFAULT_LINE_MAP: dict[str, dict] = {
    'JETSON_NANO' : {
         3: 'J3',
         5: 'J2',
         7: 'BB0',
         8: 'G0',
        10: 'G1',
        11: 'G2',
        12: 'J7',
        13: 'B6',
        15: 'Y2',
        16: 'DD0',
        18: 'B7',
        19: 'C0',
        21: 'C1',
        22: 'B5', # same as #27 # default:GPIO alt:SPI1_MISO
        23: 'C2', # same as #28 # default:GPIO alt:SPI0_SCK
        24: 'C3',
        26: 'C4',
        27: 'B5', # same as #22 # default:I2C0_SDA alt:GPIO
        28: 'C2', # same as #23 # default:I2C0_CLK alt:GPIO
        29: 'S5',
        31: 'Z0',
        32: 'V0',
        33: 'E6',
        35: 'J4',
        36: 'G3',
        37: 'B4',
        38: 'J5',
        40: 'J6'
    }
}
DEFAULT_GPIOD_CONSUMER='hx711'

class HX711:

     
    def get_line_no(self, pin_no:int) -> int:
        if not pin_no in self.line_map:
            raise RuntimeError(f"pin:{pin_no} is not found in line map.")
        line_str:str = self.line_map[pin_no]
        offset:int = int(line_str[-1])
        address:str = line_str[:-1]
        address_num:int = ord(address[0]) - ord('A')
        if len(address) == 2:
            address_num += ord('Z') - ord('A') + 1
        return address_num * 8 + offset


    def __init__(self, dout:int, pd_sck:int, gain:int = 128, mutex:bool = False, chip = None, line_map_name:str = 'JETSON_NANO', custome_line_map:dict = None):
        self.line_map = None
        if line_map_name in DEFAULT_LINE_MAP:
            self.line_map = DEFAULT_LINE_MAP[line_map_name]
        elif custome_line_map:
            self.line_map:dict[int, str] = custome_line_map
        else:
            raise RuntimeError(f"line_map_name={line_map_name} is not found. You can also specify custome_line_map for your device.")

        self.chip = chip
        if self.chip is None:
            self.chip = gpiod.Chip("0", gpiod.Chip.OPEN_BY_NUMBER)
        self.PD_SCK = self.chip.get_line(self.get_line_no(pd_sck))
        self.DOUT:int = self.chip.get_line(self.get_line_no(dout))
        self.mutex_flag:bool = mutex
        if self.mutex_flag:
            # Mutex for reading from the HX711, in case multiple threads in client
            # software try to access get values from the class at the same time.
            self.readLock = threading.Lock()
        
        self.PD_SCK.request(
            consumer=DEFAULT_GPIOD_CONSUMER,
            type=gpiod.LINE_REQ_DIR_OUT
        )
        self.DOUT.request(
            consumer=DEFAULT_GPIOD_CONSUMER,
            type=gpiod.LINE_REQ_DIR_IN
        )
        
        self.GAIN:int = 0

        # The value returned by the hx711 that corresponds to your reference
        # unit AFTER dividing by the SCALE.
        self.REFERENCE_UNIT:int = 1
        self.REFERENCE_UNIT_B:int = 1

        self.OFFSET:float = 1.0
        self.OFFSET_B:float = 1.0
        self.lastVal:float = 0.0

        self.byte_format:str = 'MSB'
        self.bit_format:str = 'MSB'

        self.set_gain(gain)

        # Think about whether this is necessary.
        time.sleep(0.1)


    def convertFromTwosComplement24bit(self, inputValue) -> int:
        return -(inputValue & 0x800000) + (inputValue & 0x7fffff)
    

    def is_ready(self) -> bool:
        return self.DOUT.get_value() == 0
    
     
    def set_gain(self, gain):
        if gain == 128:
            self.GAIN = 1
        elif gain == 64:
            self.GAIN = 3
        elif gain == 32:
            self.GAIN = 2

        self.PD_SCK.set_value(0)

        # Read out a set of raw bytes and throw it away.
        self.readRawBytes()
     
     
    def get_gain(self) -> int:
        if self.GAIN == 1:
            return 128
        if self.GAIN == 3:
            return 64
        if self.GAIN == 2:
            return 32

        # Shouldn't get here.
        return 0
     
     
    def readNextBit(self) -> int:
       # Clock HX711 Digital Serial Clock (PD_SCK).  DOUT will be
       # ready 1us after PD_SCK rising edge, so we sample after
       # lowering PD_SCL, when we know DOUT will be stable.
       self.PD_SCK.set_value(1)
       self.PD_SCK.set_value(0)
       return self.DOUT.get_value()


    def readNextByte(self) -> int:
       byteValue:int = 0

       # Read bits and build the byte from top, or bottom, depending
       # on whether we are in MSB or LSB bit mode.
       for x in range(8):
          if self.bit_format == 'MSB':
             byteValue <<= 1
             byteValue |= self.readNextBit()
          else:
             byteValue >>= 1              
             byteValue |= self.readNextBit() * 0x80

       # Return the packed byte.
       return byteValue 

     
    def readRawBytes(self) -> list[int]:
        if self.mutex_flag:
            # Wait for and get the Read Lock, incase another thread is already
            # driving the HX711 serial interface.
            self.readLock.acquire()

        # Wait until HX711 is ready for us to read a sample.
        while not self.is_ready():
           pass

        # Read three bytes of data from the HX711.
        firstByte:int  = self.readNextByte()
        secondByte:int = self.readNextByte()
        thirdByte:int  = self.readNextByte()

        # HX711 Channel and gain factor are set by number of bits read
        # after 24 data bits.
        for i in range(self.GAIN):
           # Clock a bit out of the HX711 and throw it away.
           self.readNextBit()

        if self.mutex_flag:
            # Release the Read Lock, now that we've finished driving the HX711
            # serial interface.
            self.readLock.release()           

        # Depending on how we're configured, return an orderd list of raw byte
        # values.
        if self.byte_format == 'LSB':
           return [thirdByte, secondByte, firstByte]
        else:
           return [firstByte, secondByte, thirdByte]

          
    def read_long(self) -> int:
        # Get a sample from the HX711 in the form of raw bytes.
        dataBytes:list[int] = self.readRawBytes()


        logger.debug(dataBytes,)
        
        # Join the raw bytes into a single 24bit 2s complement value.
        twosComplementValue = ((dataBytes[0] << 16) |
                               (dataBytes[1] << 8)  |
                               dataBytes[2])

        logger.debug(f"Twos: 0x{twosComplementValue:06x}")
        
        # Convert from 24bit twos-complement to a signed value.
        signedIntValue:int = self.convertFromTwosComplement24bit(twosComplementValue)

        # Record the latest sample value we've read.
        self.lastVal = signedIntValue

        # Return the sample value we've read from the HX711.
        return int(signedIntValue)
    

    def read_average(self, times:int = 3) -> float:
        # Make sure we've been asked to take a rational amount of samples.
        if times <= 0:
            raise ValueError("HX711()::read_average(): times must >= 1!!")

        # If we're only average across one value, just read it and return it.
        if times == 1:
            return self.read_long()

        # If we're averaging across a low amount of values, just take the
        # median.
        if times < 5:
            return self.read_median(times)

        # If we're taking a lot of samples, we'll collect them in a list, remove
        # the outliers, then take the mean of the remaining set.
        valueList:list[int] = []

        for x in range(times):
            valueList += [self.read_long()]

        valueList.sort()

        # We'll be trimming 20% of outlier samples from top and bottom of collected set.
        trimAmount:int = int(len(valueList) * 0.2)

        # Trim the edge case values.
        valueList:list = valueList[trimAmount:-trimAmount]

        # Return the mean of remaining samples.
        return sum(valueList) / len(valueList)

     
    # A median-based read method, might help when getting random value spikes
    # for unknown or CPU-related reasons
    def read_median(self, times:int = 3) -> float:
       if times <= 0:
          raise ValueError("HX711::read_median(): times must be greater than zero!")
      
       # If times == 1, just return a single reading.
       if times == 1:
          return self.read_long()

       valueList:list[int] = []

       for x in range(times):
          valueList += [self.read_long()]

       valueList.sort()

       # If times is odd we can just take the centre value.
       if (times & 0x1) == 0x1:
          return valueList[len(valueList) // 2]
       else:
          # If times is even we have to take the arithmetic mean of
          # the two middle values.
          midpoint:int = len(valueList) / 2
          return sum(valueList[midpoint:midpoint+2]) / 2.0

     
    # Compatibility function, uses channel A version
    def get_value(self, times:int = 3) -> float:
        return self.get_value_A(times)


    def get_value_A(self, times:int = 3) -> float:
        return self.read_median(times) - self.get_offset_A()


    def get_value_B(self, times:int = 3) -> float:
        # for channel B, we need to set_gain(32)
        g:int = self.get_gain()
        self.set_gain(32)
        value = self.read_median(times) - self.get_offset_B()
        self.set_gain(g)
        return value

     
    # Compatibility function, uses channel A version
    def get_weight(self, times:int = 3) -> float:
        return self.get_weight_A(times)


    def get_weight_A(self, times:int = 3) -> float:
        value:float = self.get_value_A(times)
        value = value / self.REFERENCE_UNIT
        return value

     
    def get_weight_B(self, times:int = 3) -> float:
        value:float = self.get_value_B(times)
        value = value / self.REFERENCE_UNIT_B
        return value
    

    # Sets tare for channel A for compatibility purposes
    def tare(self, times:int = 15) -> float:
        return self.tare_A(times)
        
    def tare_A(self, times:int = 15) -> float:
        # Backup REFERENCE_UNIT value
        backupReferenceUnit:int = self.get_reference_unit_A()
        self.set_reference_unit_A(1)
        
        value:float = self.read_average(times)
        logger.debug(f"Tare A value: {value}")
        self.set_offset_A(value)

        # Restore the reference unit, now that we've got our offset.
        self.set_reference_unit_A(backupReferenceUnit)

        return value


    def tare_B(self, times:int = 15) -> float:
        # Backup REFERENCE_UNIT value
        backupReferenceUnit:int = self.get_reference_unit_B()
        self.set_reference_unit_B(1)

        # for channel B, we need to set_gain(32)
        backupGain:int = self.get_gain()
        self.set_gain(32)

        value:float = self.read_average(times)
        logger.debug(f"Tare B value:{value}")
        self.set_offset_B(value)

        # Restore gain/channel/reference unit settings.
        self.set_gain(backupGain)
        self.set_reference_unit_B(backupReferenceUnit)
       
        return value

    
    def set_reading_format(self, byte_format:str = "LSB", bit_format:str = "MSB"):
        if byte_format == "LSB":
            self.byte_format = byte_format
        elif byte_format == "MSB":
            self.byte_format = byte_format
        else:
            raise ValueError("Unrecognised byte_format: \"%s\"" % byte_format)

        if bit_format == "LSB":
            self.bit_format = bit_format
        elif bit_format == "MSB":
            self.bit_format = bit_format
        else:
            raise ValueError("Unrecognised bitformat: \"%s\"" % bit_format)

            
    # sets offset for channel A for compatibility reasons
    def set_offset(self, offset:float):
        self.set_offset_A(offset)

    def set_offset_A(self, offset:float):
        self.OFFSET = offset

    def set_offset_B(self, offset:float):
        self.OFFSET_B = offset

    def get_offset(self) -> float:
        return self.get_offset_A()

    def get_offset_A(self) -> float:
        return self.OFFSET

    def get_offset_B(self) -> float:
        return self.OFFSET_B


    def set_reference_unit(self, reference_unit:int):
        self.set_reference_unit_A(reference_unit)

        
    def set_reference_unit_A(self, reference_unit:int):
        # Make sure we aren't asked to use an invalid reference unit.
        if reference_unit == 0:
            raise ValueError("HX711::set_reference_unit_A() can't accept 0 as a reference unit!")
            return

        self.REFERENCE_UNIT = reference_unit

        
    def set_reference_unit_B(self, reference_unit:int):
        # Make sure we aren't asked to use an invalid reference unit.
        if reference_unit == 0:
            raise ValueError("HX711::set_reference_unit_A() can't accept 0 as a reference unit!")
            return

        self.REFERENCE_UNIT_B = reference_unit


    def get_reference_unit(self) -> int:
        return self.get_reference_unit_A()

        
    def get_reference_unit_A(self) -> int:
        return self.REFERENCE_UNIT

        
    def get_reference_unit_B(self) -> int:
        return self.REFERENCE_UNIT_B
        
        
    def power_down(self):
        if self.mutex_flag:
            # Wait for and get the Read Lock, incase another thread is already
            # driving the HX711 serial interface.
            self.readLock.acquire()

        # Cause a rising edge on HX711 Digital Serial Clock (PD_SCK).  We then
        # leave it held up and wait 100 us.  After 60us the HX711 should be
        # powered down.
        self.PD_SCK.set_value(0)
        self.PD_SCK.set_value(1)

        time.sleep(0.0001)

        if self.mutex_flag:
            # Release the Read Lock, now that we've finished driving the HX711
            # serial interface.
            self.readLock.release()           


    def power_up(self):
        if self.mutex_flag:
            # Wait for and get the Read Lock, incase another thread is already
            # driving the HX711 serial interface.
            self.readLock.acquire()

        # Lower the HX711 Digital Serial Clock (PD_SCK) line.
        self.PD_SCK.set_value(0)

        # Wait 100 us for the HX711 to power back up.
        time.sleep(0.0001)

        if self.mutex_flag:
            # Release the Read Lock, now that we've finished driving the HX711
            # serial interface.
            self.readLock.release()

        # HX711 will now be defaulted to Channel A with gain of 128.  If this
        # isn't what client software has requested from us, take a sample and
        # throw it away, so that next sample from the HX711 will be from the
        # correct channel/gain.
        if self.get_gain() != 128:
            self.readRawBytes()


    def reset(self):
        self.power_down()
        self.power_up()
