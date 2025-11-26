# Step 1: Install and Test clang-format 

Simple setup for these platforms: macOS and Ubuntu

---

## Install clang-format 

### macOS
'''bash
brew install clang-format
'''

### Ubuntu 
'''bash
sudo apt-get update
sudo apt-get install clang-format
'''

### Verify Install
'''bash
clang-format --version 
'''

## Test clang-format 

### Apply formatting 
'''bash 
clang-format -i file.cpp
'''