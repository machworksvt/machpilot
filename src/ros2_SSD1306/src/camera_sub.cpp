#include <memory>
#include <bcm2835.h>
#include <cstdio>
#include <SSD1306_OLED.hpp>
#include <string>
#include <controller.h>
using std::placeholders::_1;

#define myOLEDwidth  128
#define myOLEDheight 64
#define FULLSCREEN (myOLEDwidth * (myOLEDheight/8))
SSD1306 myOLED(myOLEDwidth, myOLEDheight) ; // instantiate a OLED object

bool SetupTest()
{
	const uint16_t I2C_Speed = BCM2835_I2C_CLOCK_DIVIDER_626; //  bcm2835I2CClockDivider enum , see readme.
	const uint8_t I2C_Address = 0x3C;
	bool I2C_debug = false;
	printf("OLED Test Begin\r\n");

	// Check if Bcm28235 lib installed and print version.
	if(!bcm2835_init())
	{
		printf("Error 1201: init bcm2835 library , Is it installed ?\r\n");
		return false;
	}

	// Turn on I2C bus (optionally it may already be on)
	if(!myOLED.OLED_I2C_ON())
	{
		printf("Error 1202: bcm2835_i2c_begin :Cannot start I2C, Running as root?\n");
		bcm2835_close(); // Close the library
		return false;
	}

	printf("SSD1306 library Version Number :: %u\r\n",myOLED.getLibVerNum());
	printf("bcm2835 library Version Number :: %u\r\n",bcm2835_version());
	bcm2835_delay(500);
	myOLED.OLEDbegin(I2C_Speed, I2C_Address, I2C_debug); // initialize the OLED
	myOLED.OLEDFillScreen(0xF0, 0); // splash screen bars, optional just for effect
	bcm2835_delay(1000);
	return true;
}

void EndTest()
{
	myOLED.OLEDPowerDown(); //Switch off display
	myOLED.OLED_I2C_OFF(); // Switch off I2C , optional may effect other programs & devices
	bcm2835_close(); // Close the library
	printf("OLED Test End\r\n");
}

void TestLoop(std::string message)
{

	// Define a buffer to cover whole screen
	uint8_t  screenBuffer[FULLSCREEN];
	if (!myOLED.OLEDSetBufferPtr(myOLEDwidth, myOLEDheight, screenBuffer, sizeof(screenBuffer))) return;
	myOLED.OLEDclearBuffer();
	myOLED.setTextColor(WHITE);
	myOLED.setCursor(10, 10);
	myOLED.print(message);
	myOLED.OLEDupdate();
	delay(5000);
}

class DisplaySubNode : public rclcpp::Node
{
    public:
    DisplaySubNode()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Temperature>(
      "temp", 10, std::bind(&DisplaySubNode::topic_callback, this, _1));
    }

    private:
    void topic_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) const
    {
		std::string message = "The temperature is: Joe";
        TestLoop(message);
    }
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if(!SetupTest()) return -1;
    rclcpp::spin(std::make_shared<DisplaySubNode>());
    EndTest();
    rclcpp::shutdown();
    return 0;
}