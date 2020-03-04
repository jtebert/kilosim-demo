# Compile your code with:
# mkdir build
# cd build
# cmake ..
# make

cmake_minimum_required (VERSION 3.9)

project(kilosim_demo LANGUAGES CXX)

# Change this to the path of your local installation of Kilosim
add_subdirectory(/home/jtebert/google-drive/Harvard/SSR/kilobots/kilosim-richard-make build)

add_executable(kilosim_demo src/BayesBot.cpp)
target_link_libraries(kilosim_demo PRIVATE kilosim)

