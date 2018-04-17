add_library(gridfastslam SHARED
    gfsreader.cpp
    gridslamprocessor.cpp
    gridslamprocessor_tree.cpp
    motionmodel.cpp
)
target_link_libraries(gridfastslam scanmatcher sensor_range)

install(TARGETS gridfastslam LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
