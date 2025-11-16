#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

# HX711 library for Jetson Orin Nano using libgpiod v1.6.3
import sys
sys.path.insert(0, '/usr/local/lib/python3.10/site-packages')  # Use libgpiod v1.6

try:
    import gpiod
    from hx711 import HX711
    HX711_AVAILABLE = True
except ImportError as e:
    HX711_AVAILABLE = False
    print(f"Warning: HX711 library not available: {e}")

# Create a dummy class for simulation/build time
class HX711Simulator:
    def __init__(self, *args, **kwargs):
        self.simulated = True
        self.reference_unit = 1
        self.offset = 0
        print("HX711: Running in simulation mode")
    
    def set_reading_format(self, byte_format="MSB", bit_format="MSB"):
        pass
    
    def set_reference_unit(self, reference_unit):
        self.reference_unit = reference_unit
    
    def reset(self):
        pass
    
    def tare(self, readings=15):
        return True
    
    def get_raw_data_mean(self, readings=1):
        import random
        return int(random.uniform(-8388608, 8388607))
    
    def get_weight(self, readings=5):
        import random
        # Return simulated weight in grams
        raw_value = self.get_raw_data_mean(readings)
        weight = (raw_value - self.offset) / self.reference_unit
        return weight + random.uniform(-0.1, 0.1)  # Add some noise
    
    def power_down(self):
        pass
    
    def power_up(self):
        pass

# If HX711 not available, use simulator
if not HX711_AVAILABLE:
    HX711 = HX711Simulator
    def __init__(self, dout, pd_sck, chip=None, **kwargs):
        self.simulated = False
        self.dout_pin = dout
        self.pd_sck_pin = pd_sck
        self._lock = threading.Lock()
        self.reference_unit = 1
        self.offset = 0
        self.byte_format = "MSB"
        self.bit_format = "MSB"
        
        if chip is None and GPIOD_AVAILABLE:
            # Create our own chip instance using modern gpiod API
            try:
                self.chip = gpiod.Chip(path="/dev/gpiochip0")
                self.chip_owned = True
            except Exception as e:
                raise RuntimeError(f"Failed to open GPIO chip: {e}")
        elif chip is not None:
            self.chip = chip
            self.chip_owned = False
        else:
            raise ImportError("gpiod library not available")
        
        try:
            # Configure GPIO lines using modern gpiod v2 API
            config = {
                self.dout_pin: gpiod.LineSettings(direction=gpiod.line.Direction.INPUT),
                self.pd_sck_pin: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE)
            }
            
            self.request = self.chip.request_lines(config=config)
            
        except Exception as e:
            if self.chip_owned:
                self.chip.close()
            raise RuntimeError(f"Failed to configure GPIO lines: {e}")
    
    def set_reading_format(self, byte_format="MSB", bit_format="MSB"):
        """Set the reading format for data interpretation"""
        self.byte_format = byte_format
        self.bit_format = bit_format
    
    def set_reference_unit(self, reference_unit):
        """Set the reference unit for weight calculations"""
        self.reference_unit = reference_unit
    
    def get_weight(self, readings=5):
        """Get weight in grams using reference unit and offset"""
        raw_value = self.get_raw_data_mean(readings)
        if raw_value is False:
            return False
        
        weight = (raw_value - self.offset) / self.reference_unit
        return weight
    
    def tare(self, readings=15):
        """Tare the scale (set current reading as zero point)"""
        raw_value = self.get_raw_data_mean(readings)
        if raw_value is not False:
            self.offset = raw_value
            return True
        return False
    
    def is_ready(self):
        """Check if HX711 is ready for reading"""
        try:
            dout_val = self.request.get_value(self.dout_pin)
            return dout_val == gpiod.line.Value.INACTIVE
        except:
            return False
    
    def wait_ready(self, timeout=1.0):
        """Wait for HX711 to be ready"""
        start_time = time.time()
        while not self.is_ready():
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.001)
        return True
    
    def read_raw(self):
        """Read raw 24-bit value from HX711"""
        with self._lock:
            if not self.wait_ready():
                raise RuntimeError("HX711 not ready for reading")
            
            # Read 24 bits
            data = 0
            for i in range(24):
                # Clock high
                self.request.set_value(self.pd_sck_pin, gpiod.line.Value.ACTIVE)
                time.sleep(0.000001)  # 1us delay
                
                # Read data bit
                if self.request.get_value(self.dout_pin) == gpiod.line.Value.ACTIVE:
                    data |= (1 << (23 - i))
                
                # Clock low
                self.request.set_value(self.pd_sck_pin, gpiod.line.Value.INACTIVE)
                time.sleep(0.000001)  # 1us delay
            
            # Send gain pulse (for channel A, gain 128)
            self.request.set_value(self.pd_sck_pin, gpiod.line.Value.ACTIVE)
            time.sleep(0.000001)
            self.request.set_value(self.pd_sck_pin, gpiod.line.Value.INACTIVE)
            time.sleep(0.000001)
            
            # Convert to signed 24-bit value
            if data & 0x800000:  # Check sign bit
                data = data - 0x1000000
            
            return data
    
    def get_raw_data_mean(self, readings=1):
        """Get average of multiple readings"""
        if readings <= 0:
            return False
        
        total = 0
        successful_reads = 0
        
        for _ in range(readings):
            try:
                value = self.read_raw()
                total += value
                successful_reads += 1
            except:
                pass
        
        if successful_reads == 0:
            return False
        
        return total // successful_reads
    
    def reset(self):
        """Reset the HX711"""
        with self._lock:
            # Pull PD_SCK high for >60us to reset
            self.request.set_value(self.pd_sck_pin, gpiod.line.Value.ACTIVE)
            time.sleep(0.0001)  # 100us
            self.request.set_value(self.pd_sck_pin, gpiod.line.Value.INACTIVE)
            time.sleep(0.01)  # Wait for stabilization
    
    def power_down(self):
        """Power down the HX711"""
        try:
            self.request.set_value(self.pd_sck_pin, gpiod.line.Value.ACTIVE)
        except:
            pass
    
    def __del__(self):
        """Cleanup GPIO resources"""
        try:
            if hasattr(self, 'request'):
                self.request.release()
            if hasattr(self, 'chip'):
                self.chip.close()
        except:
            pass

# Create a dummy class for simulation/build time
class HX711Simulator:
    def __init__(self, *args, **kwargs):
        self.simulated = True
        self.reference_unit = 1
        self.offset = 0
        print("HX711: Running in simulation mode")
    
    def set_reading_format(self, byte_format="MSB", bit_format="MSB"):
        pass
    
    def set_reference_unit(self, reference_unit):
        self.reference_unit = reference_unit
    
    def reset(self):
        pass
    
    def tare(self, readings=15):
        return True
    
    def get_raw_data_mean(self, readings=1):
        import random
        return int(random.uniform(-8388608, 8388607))
    
    def get_weight(self, readings=5):
        import random
        # Return simulated weight in grams
        raw_value = self.get_raw_data_mean(readings)
        weight = (raw_value - self.offset) / self.reference_unit
        return weight + random.uniform(-0.1, 0.1)  # Add some noise
    
    def power_down(self):
        pass
    
    def power_up(self):
        pass

# If gpiod is not available, use simulator
if not GPIOD_AVAILABLE:
    HX711 = HX711Simulator

class HX711Node(Node):
    """
    ROS2 node for HX711 load cell amplifier.
    Publishes weight measurements as Float32 messages.
    """
    
    def __init__(self):
        super().__init__('hx711_node')
        
        # Declare parameters for GPIO pins
        self.declare_parameter('dout_pin', 11)  # GPIO pin for HX711 DOUT (data) - Physical pin 23
        self.declare_parameter('pd_sck_pin', 7)  # GPIO pin for HX711 PD_SCK (clock) - Physical pin 26
        self.declare_parameter('gain', 128)  # HX711 gain setting
        self.declare_parameter('calibration_factor', 1.0)  # Calibration factor
        self.declare_parameter('offset', 0.0)  # Offset/tare value
        self.declare_parameter('read_frequency', 10.0)  # Reading frequency in Hz
        
        # Get parameters
        self.dout_pin = self.get_parameter('dout_pin').get_parameter_value().integer_value
        self.pd_sck_pin = self.get_parameter('pd_sck_pin').get_parameter_value().integer_value
        self.gain = self.get_parameter('gain').get_parameter_value().integer_value
        self.calibration_factor = self.get_parameter('calibration_factor').get_parameter_value().double_value
        self.offset = self.get_parameter('offset').get_parameter_value().double_value
        read_frequency = self.get_parameter('read_frequency').get_parameter_value().double_value
        
        # Create publisher for weight data
        self.weight_publisher = self.create_publisher(
            Float32, 
            'weight', 
            10
        )
        
        # Initialize HX711 hardware
        self.hx711 = None
        self.initialize_hx711()
        
        # Create timer for periodic weight readings
        self.timer_period = 1.0 / read_frequency
        self.timer = self.create_timer(
            self.timer_period, 
            self.timer_callback
        )
        
        self.get_logger().info(f'HX711 node started on pins DOUT={self.dout_pin}, PD_SCK={self.pd_sck_pin}')
    
    def initialize_hx711(self):
        """
        Initialize the HX711 sensor following the hx711py-jetsonnano pattern.
        """
        try:
            # Create GPIO chip if gpiod is available
            chip = None
            if GPIOD_AVAILABLE:
                try:
                    chip = gpiod.Chip(path="/dev/gpiochip0")
                except Exception as e:
                    self.get_logger().error(f'Failed to open GPIO chip: {e}')
                    chip = None
            
            # Initialize HX711 with chip parameter
            self.hx711 = HX711(
                dout=self.dout_pin,
                pd_sck=self.pd_sck_pin,
                chip=chip
            )
            
            # Only reset and configure if not in simulation mode
            if hasattr(self.hx711, 'simulated') and self.hx711.simulated:
                self.get_logger().info('HX711 running in simulation mode')
            else:
                # Configure the HX711 like in the example
                self.hx711.set_reading_format("MSB", "MSB")
                self.hx711.set_reference_unit(self.calibration_factor)
                self.hx711.reset()
                # Wait for HX711 to stabilize
                time.sleep(0.5)
                # Tare the scale
                self.hx711.tare()
                self.get_logger().info(f'HX711 initialized successfully with real hardware on pins DOUT={self.dout_pin}, PD_SCK={self.pd_sck_pin}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize HX711: {str(e)}')
            # Fall back to simulator
            self.hx711 = HX711Simulator()
            self.get_logger().warn('Falling back to simulation mode')
    
    def read_raw_weight(self):
        """
        Read weight data from HX711 using the new get_weight method.
        Returns weight in grams or simulated data if hardware not available.
        """
        if self.hx711 is None:
            # Return simulated data if hardware not available
            import random
            simulated_weight = random.uniform(-100, 100)
            return simulated_weight
        
        try:
            # Use the new get_weight method which returns calibrated weight
            weight = self.hx711.get_weight(5)  # Average 5 readings
            
            if weight is False:
                self.get_logger().warn('HX711 reading failed - sensor not ready')
                return None
                
            return weight
            
        except Exception as e:
            self.get_logger().error(f'Error reading HX711: {str(e)}')
            # If real hardware fails, return simulated data to keep node running
            import random
            simulated_weight = random.uniform(-100, 100)
            self.get_logger().warn(f'Using simulated data due to hardware error: {simulated_weight}')
            return simulated_weight
    
    def convert_to_weight(self, raw_value):
        """
        Convert raw HX711 reading to actual weight in grams.
        """
        if raw_value is None:
            return None
            
        weight = (raw_value - self.offset) / self.calibration_factor
        return weight
    
    def timer_callback(self):
        """
        Timer callback to read and publish weight data.
        """
        try:
            # Read weight directly (already calibrated)
            weight = self.read_raw_weight()
            
            if weight is None:
                return  # Skip this reading if sensor not ready
            
            # Create and publish message
            msg = Float32()
            msg.data = float(weight)
            self.weight_publisher.publish(msg)
            
            # Log occasional readings for debugging
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
                
            if self._log_counter % 50 == 0:  # Log every 5 seconds at 10Hz
                self.get_logger().info(f'Weight: {weight:.2f} g')
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
    
    def destroy_node(self):
        """
        Clean up resources when the node is destroyed.
        """
        try:
            if hasattr(self, 'hx711') and self.hx711:
                if hasattr(self.hx711, 'power_down') and not hasattr(self.hx711, 'simulated'):
                    self.hx711.power_down()
                    self.get_logger().info('HX711 powered down')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')
        
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the HX711 node.
    """
    rclpy.init(args=args)
    
    hx711_node = None
    try:
        hx711_node = HX711Node()
        rclpy.spin(hx711_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if hx711_node:
            hx711_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()