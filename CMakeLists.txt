# Compile your code with: (see README for additional information)
# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make

cmake_minimum_required (VERSION 3.9)

project(kilosim_demo LANGUAGES CXX C)

# Identify the location of the kilosim library
add_subdirectory(submodules/kilosim)

# Directory containing header files
include_directories(api)

# Be sure to list all source files
add_executable(kilosim_demo
  src/BayesBot.cpp
  src/incbeta.c
  src/main.cpp
)

# Link the kilosim library
target_link_libraries(kilosim_demo PRIVATE kilosim)
