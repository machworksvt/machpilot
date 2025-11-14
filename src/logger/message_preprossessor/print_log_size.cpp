#include <iostream>
#include "../src/message.hpp"


int main() {
    std::cout << "uint8["<<sizeof(Log)<<"] data";
    

    //segmentation fault so I can tell if the code generator is running
    int *ptr = NULL;
    *ptr = 10; 
    
    return 0;
}