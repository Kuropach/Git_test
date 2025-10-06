cmake_minimum_required(VERSION 3.10)
project(MyJsonProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the JSON package
find_package(nlohmann_json 3.2.0 REQUIRED)

# Create executable
add_executable(my_app src/main.cpp)

# Link JSON library
target_link_libraries(my_app nlohmann_json::nlohmann_json)