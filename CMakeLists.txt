cmake_minimum_required(VERSION 3.15)
project(mover VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ethercat -------------------------------

find_library(EtherCAT_LIBRARY
    NAMES ethercat
    PATHS /usr/local/lib
)
find_path(EtherCAT_INCLUDE_DIR
    NAMES ecrt.h
    PATHS /usr/local/include
)

mark_as_advanced(EtherCAT_LIBRARY EtherCAT_INCLUDE_DIR)
if(NOT TARGET EtherLab::ethercat)
    add_library(EtherLab::ethercat SHARED IMPORTED)
    set_target_properties(EtherLab::ethercat PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EtherCAT_INCLUDE_DIR}"
        IMPORTED_LOCATION "${EtherCAT_LIBRARY}"
    )
endif()

#-------------------------------------------

find_package(gravity_util_lib CONFIG REQUIRED)

file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    EtherLab::ethercat
    gravity::gravity_util_lib

)


# check usage:  sudo du -h -d1 . | sort -h 
# du -h -d1 
# check free:  sudo df -h . | sort -h
# du -h -d 1 vcpkg/installed/
# rm -rf ~/vcpkg/buildtrees/* ~/vcpkg/packages/* ~/vcpkg/downloads/*
# vcpkg install grpc protobuf