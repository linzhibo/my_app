# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )

#find_package(Boost REQUIRED)
set(Boost_INCLUDE_DIRS /home/zhibo/Downloads/boost_1_67_0)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(src/main/cpp/include)

add_subdirectory(src/main/cpp/lib/gridfastslam)
add_subdirectory(src/main/cpp/lib/scanmatcher)
add_subdirectory(src/main/cpp/lib/sensor)
add_subdirectory(src/main/cpp/lib/utils)

add_library( # Sets the name of the library.
             slam-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/src/main.cpp
             src/main/cpp/src/gslam.cpp)

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       slam-lib
                       gridfastslam
                       scanmatcher
                       utils
                       sensor_base
                       sensor_odometry
                       sensor_range
                       #${Boost_LIBRARIES}
                       # Links the target library to the log library
                       # included in the NDK.

                       ${log-lib} )