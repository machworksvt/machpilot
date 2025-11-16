#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import sys

# HX711 library for Jetson Orin Nano using libgpiod v1.6.3
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

class HX711Node(Node):
    """
    ROS2 node for HX711 load cell amplifier.
    Publishes weight measurements as Float32 messages.
    """
    
    def __init__(self):
        super().__init__('hx711_node')
        
        # Declare parameters for GPIO pins (using available GPIO numbers)
        self.declare_parameter('dout_pin', 9)  # Physical pin 9 -> GPIO line 9 (DOUT - data input)
        self.declare_parameter('pd_sck_pin', 11)  # Physical pin 11 -> GPIO line 11 (PD_SCK - clock output)
        self.declare_parameter('gain', 128)  # HX711 gain setting
        self.declare_parameter('calibration_factor', 1.0)  # Calibration factor
        self.declare_parameter('offset', 0.0)  # Offset/tare value
        self.declare_parameter('read_frequency', 1.0)  # Reading frequency in Hz (slower for stability)
        
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
        self.gpio_chip = None
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
            # Create GPIO chip if available - match the example exactly
            self.gpio_chip = None
            if HX711_AVAILABLE:
                try:
                    self.gpio_chip = gpiod.Chip("0", gpiod.Chip.OPEN_BY_NUMBER)
                    self.get_logger().info('GPIO chip opened successfully')
                except Exception as e:
                    self.get_logger().error(f'Failed to open GPIO chip: {e}')
                    self.gpio_chip = None
            
            # Initialize HX711 with chip parameter - use direct GPIO line mapping for Jetson Orin Nano
            # The original library's calculation doesn't work for Orin Nano
            # Let's use a simpler approach with direct line numbers that we know exist
            orin_nano_line_map = {
                9: "A9",    # Physical pin 9 -> Use format that results in GPIO line 9 (DOUT)
                11: "B3",   # Physical pin 11 -> Use format that results in GPIO line 11 (PD_SCK)
            }
            
            self.hx711 = HX711(
                dout=self.dout_pin,
                pd_sck=self.pd_sck_pin,
                chip=self.gpio_chip,
                line_map_name=None,  # Disable default JETSON_NANO mapping
                custome_line_map=orin_nano_line_map  # Use custom mapping
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
                if self.hx711.tare():
                    self.get_logger().info('HX711 tared successfully')
                else:
                    self.get_logger().warn('HX711 tare failed')
                self.get_logger().info(f'HX711 initialized successfully with real hardware on pins DOUT={self.dout_pin}, PD_SCK={self.pd_sck_pin}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize HX711: {str(e)}')
            # Fall back to simulator
            self.hx711 = HX711Simulator()
            self.get_logger().warn('Falling back to simulation mode')
    
    def read_weight(self):
        """
        Read weight data from HX711 using the get_weight method.
        Returns weight in grams or simulated data if hardware not available.
        """
        if self.hx711 is None:
            return None
        
        try:
            # Use the get_weight method which returns calibrated weight
            weight = self.hx711.get_weight(5)  # Average 5 readings
            
            if weight is False:
                self.get_logger().warn('HX711 reading failed - sensor not ready')
                return None
                
            return weight
            
        except Exception as e:
            self.get_logger().error(f'Error reading HX711: {str(e)}')
            return None
    
    def timer_callback(self):
        """
        Timer callback to read and publish weight data.
        """
        try:
            # Read weight directly (already calibrated)
            weight = self.read_weight()
            
            if weight is None:
                return  # Skip this reading if sensor not ready
            
            # Create and publish message
            msg = Float32()
            msg.data = float(weight)
            self.weight_publisher.publish(msg)
            
            # Log readings
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
            
            # Close GPIO chip like in the example
            if hasattr(self, 'gpio_chip') and self.gpio_chip:
                self.gpio_chip.close()
                self.get_logger().info('GPIO chip closed')
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