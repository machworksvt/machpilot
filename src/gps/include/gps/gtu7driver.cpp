#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <string>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>

// Structure to hold parsed GPS data
struct GPSData {
    double latitude;
    double longitude;
    int year, month, day;
    int hour, minute, second;
    bool valid;
};

// Validate NMEA sentence checksum.
// The sentence should start with '$' and contain a '*' before the checksum.
bool validateChecksum(const std::string &sentence) {
    if (sentence.size() < 9 || sentence[0] != '$') return false;
    size_t asteriskPos = sentence.find('*');
    if (asteriskPos == std::string::npos) return false;

    unsigned char checksum = 0;
    for (size_t i = 1; i < asteriskPos; i++) {
        checksum ^= sentence[i];
    }

    std::string checksumStr = sentence.substr(asteriskPos + 1, 2);
    unsigned int providedChecksum;
    std::istringstream iss(checksumStr);
    iss >> std::hex >> providedChecksum;
    return (checksum == providedChecksum);
}

// Convert NMEA coordinate (ddmm.mmmm or dddmm.mmmm) to decimal degrees.
double convertToDecimalDegrees(const std::string &coord, const std::string &direction) {
    if(coord.empty() || direction.empty())
        return 0.0;
    // Determine degrees length based on whether it's latitude or longitude.
    size_t degLength = (direction == "N" || direction == "S") ? 2 : 3;
    double degrees = std::stod(coord.substr(0, degLength));
    double minutes = std::stod(coord.substr(degLength));
    double decimal = degrees + minutes / 60.0;
    if(direction == "S" || direction == "W")
        decimal = -decimal;
    return decimal;
}

// Parse a GPRMC sentence and extract relevant GPS data.
GPSData parseGPRMC(const std::string &sentence) {
    GPSData data = {};
    data.valid = false;

    if(!validateChecksum(sentence))
        return data;

    std::istringstream ss(sentence);
    std::string token;
    std::vector<std::string> tokens;
    while(std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    // Check that we have the minimum number of tokens.
    if(tokens.size() < 10 || tokens[0].substr(1) != "GPRMC")
        return data;

    // tokens[2] should be 'A' for valid data.
    if(tokens[2] != "A")
        return data;

    // Ensure the expected tokens are present.
    if(tokens[1].size() < 6 || tokens[3].empty() || tokens[4].empty() ||
       tokens[5].empty() || tokens[6].empty() || tokens[9].size() != 6)
       return data;

    try {
        // Parse time (hhmmss.sss)
        data.hour = std::stoi(tokens[1].substr(0, 2));
        data.minute = std::stoi(tokens[1].substr(2, 2));
        data.second = std::stoi(tokens[1].substr(4, 2));

        // Parse date (ddmmyy)
        data.day = std::stoi(tokens[9].substr(0, 2));
        data.month = std::stoi(tokens[9].substr(2, 2));
        data.year = std::stoi(tokens[9].substr(4, 2)) + 2000; // Assumes 21st century

        // Parse latitude and longitude.
        data.latitude = convertToDecimalDegrees(tokens[3], tokens[4]);
        data.longitude = convertToDecimalDegrees(tokens[5], tokens[6]);

        data.valid = true;
    } catch (...) {
        data.valid = false;
    }

    return data;
}

// Function to display parsed GPS data.
void displayInfo(const GPSData &data) {
    if (data.valid) {
        std::cout << "Location: " 
                  << std::fixed << std::setprecision(6) << data.latitude 
                  << ", " << data.longitude << "  Date/Time: "
                  << data.month << "/" << data.day << "/" << data.year << " ";
        std::cout << (data.hour < 10 ? "0" : "") << data.hour << ":"
                  << (data.minute < 10 ? "0" : "") << data.minute << ":"
                  << (data.second < 10 ? "0" : "") << data.second << std::endl;
    } else {
        std::cout << "Invalid or incomplete data received." << std::endl;
    }
}

int main() {
    // Change the serial port name as appropriate for your system.
    std::string portName = "/dev/ttyTHS1";
    unsigned int baudRate = 9600;

    std::cout << "Running" << std::endl;

    boost::asio::io_service io;
    boost::asio::serial_port serial(io);

    try {
        serial.open(portName);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        std::cout << "Port open" << std::endl;
    } catch (boost::system::system_error &e) {
        std::cerr << "Error opening serial port " << portName << ": " << e.what() << std::endl;
        return 1;
    }

    std::string buffer;
    char c;
    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        boost::asio::read(serial, boost::asio::buffer(&c, 1));
        std::cout << c << std::endl;
        if (c == '\n') {
            // Remove potential trailing carriage return.
            if (!buffer.empty() && buffer.back() == '\r') {
                std::cout << "Buffer not empty" << std::endl;
                buffer.pop_back();
            }
            // If this is a GPRMC sentence, parse and display it.
            if (buffer.rfind("$GPRMC", 0) == 0) {
                std::cout << "Found a line" << std::endl;
                GPSData data = parseGPRMC(buffer);
                displayInfo(data);
            }
            buffer.clear();
        } else {
            buffer.push_back(c);
        }

        // If no valid GPS data is received within 5 seconds, report an error.
        auto currentTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count() > 5000) {
            if(buffer.empty()) {
                std::cout << "No GPS data detected: check wiring or connection." << std::endl;
                break;
            }
        }
    }
    return 0;
}
