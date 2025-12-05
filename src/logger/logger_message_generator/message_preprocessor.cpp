#include <iostream>
#include "../logger_message/src/message.hpp"
#include <fstream>


int main(int argc, char* argv[]) {
    
    if (argc!=3){
        std::cerr<<"message_preprocessor takes in two arguments message location and test location\n";
        return 1;
    }

    std::ofstream message_file(argv[1]);
    std::ofstream    test_file(argv[2]);

    message_file<< "uint8["<<sizeof(Log)<<"] data";
    
    test_file <<
        "#include \"message.hpp\"\n"
        "static_assert(sizeof(Log)=="<<sizeof(Log)<<",\"the size of Log changed run \\\"colcon build --packages-select logger_message_generator\\\" to fix this\");";



    return 0;
}