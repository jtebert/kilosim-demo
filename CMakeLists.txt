# Compile your code with:
# mkdir build
# cd build
# cmake ..
# make

cmake_minimum_required (VERSION 3.9)

project(project_name LANGUAGES CXX)

add_subdirectory(/home/jtebert/google-drive/Harvard/SSR/kilobots/kilosim-richard-make build)

add_executable(project_name src/BayesBot.cpp)
target_link_libraries(project_name PRIVATE kilosim)

