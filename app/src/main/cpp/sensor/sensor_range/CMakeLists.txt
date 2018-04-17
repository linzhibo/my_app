add_library(sensor_range SHARED rangereading.cpp rangesensor.cpp)
target_link_libraries(sensor_range sensor_base)


