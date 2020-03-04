# Compile your code with:
# mkdir build
# cd build
# cmake ..
# make

cmake_minimum_required (VERSION 3.9)

project(kilosim_demo LANGUAGES CXX C)

# Change this to the path of your local installation of Kilosim
add_subdirectory(/home/jtebert/google-drive/Harvard/SSR/kilobots/kilosim-richard-make build)


add_executable(kilosim_demo src/main.cpp src/BayesBot.cpp src/incbeta.c)
target_link_libraries(kilosim_demo PRIVATE kilosim)


