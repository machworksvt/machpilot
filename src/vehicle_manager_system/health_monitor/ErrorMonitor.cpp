#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class ErrorMonitorNode : public rclcpp::Node
{
  public:
    ErrorMonitorNode()
    : Node("error_monitor")
    {
        publisher = this->create_publisher<std_msgs::msg::String>("temp", 10);
        
        subscription = this->create_subscription<std_msgs::msg::String>(
            "errors", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::string sendError = read_csv(*msg);
                std_msgs::msg::String out;
                out.data = sendError;
                publisher->publish(out);
                RCLCPP_INFO(this->get_logger(), "Published: %s", out.data.c_str());
            }
        );
    }

    private:
        static const std::string sensors[6];    
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;  

    private:
        std::string read_csv(const std_msgs::msg::String & msg) {
            std::string eMessage = "Error: ";
            
            std::fstream fin("example_error_codes.csv");
            if(!(fin.is_open())){
                return "Could not open csv file.";
            }

            int sensor = -1;
            int instance = -1;
            int errorNum = -1;
            std::istringstream iss(msg.data);
            if(!(iss >> sensor >> instance >> errorNum)){
                return "Input not 3 integers.";
            };

            int sCheck, iCheck, eCheck;
            std::vector<std::string> row;
            std::string line, word, temp;

            while(std::getline(fin, line)){
                row.clear();
                std::stringstream s(line);
                while (std::getline(s, word, ','))
                {
                    row.push_back(word);
                }
                sCheck = std::stoi(row[0]);
                iCheck = std::stoi(row[1]);
                eCheck = std::stoi(row[2]);
                if(sCheck==sensor && iCheck==instance && eCheck==errorNum){
                    eMessage += sensors[sensor] + " #" + row[1] + ", Severity: " + row[3] + ", " + row[4];
                    return eMessage;
                }
            }
            eMessage += "Could not find the error code in the csv file.";
            fin.close();
            return eMessage;
        }
};

const std::string ErrorMonitorNode::sensors[6] = {
    "Engine", "Radio", "GPS", "Servos", "IMU", "Pitot tubes"
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorMonitorNode>());
    rclcpp::shutdown();
    return 0;
}