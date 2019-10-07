cmake_minimum_required(VERSION 2.8.3)
project(point_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    random_numbers
    message_generation
    )
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        std_msgs
        geometry_msgs
        random_numbers
)
include_directories(include)

set(LIBS
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_lib
    src/PointGenerator.cpp
)
add_dependencies(
    ${PROJECT_NAME}_lib
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_lib ${LIBS})
set(LIBS ${PROJECT_NAME}_lib ${LIBS})

add_executable(${PROJECT_NAME}_node src/point_generator_node.cpp)
add_dependencies(
    ${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node ${LIBS})


## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
        PATTERN ".svn" EXCLUDE)


