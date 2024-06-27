find_package(OpenCV 4 REQUIRED)
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    
)
add_library(correct src/corrector.cpp)
target_link_libraries(correct
    ${OpenCV_LIBS}

)