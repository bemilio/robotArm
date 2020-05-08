to cross-compile run

cmake -D CMAKE_TOOLCHAIN_FILE=/path/to/Arduino-toolchain.cmake <CMAKE_SOURCE_DIR>

remember to set up the arduino IDE location using ccmake

The toolchain will then create a BoardOptions.cmake file in the build folder: uncmment the board you need
