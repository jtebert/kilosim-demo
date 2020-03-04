# Compile your code with:
# mkdir build
# cd build
# cmake ..
# make

cmake_minimum_required (VERSION 3.9)

project(kilosim_demo LANGUAGES CXX C)

# Change this to the path of your local installation of Kilosim
add_subdirectory(/home/jtebert/google-drive/Harvard/SSR/kilobots/kilosim-richard-make build)

file(GLOB_RECURSE SOURCES RELATIVE ${PROJECT_SOURCE_DIR} "src/*.cpp")
file(GLOB_RECURSE HEADERS RELATIVE ${PROJECT_SOURCE_DIR} "src/*.h")
file(GLOB_RECURSE CFILES RELATIVE ${PROJECT_SOURCE_DIR} "src/*.c")

add_executable(kilosim_demo ${HEADERS} ${SOURCES} ${CFILES})
# add_executable(kilosim_demo src/main.cpp)
target_link_libraries(kilosim_demo PRIVATE kilosim)
