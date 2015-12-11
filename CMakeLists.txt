cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcapVisualizer_example)

file(GLOB_RECURSE PCAP_SOURCES "src/*.cpp")
file(GLOB_RECURSE PCAP_HEADERS "src/*.h")

set (PCAP_INCLUDE_DIRS "")
foreach (_headerFile ${PCAP_HEADERS})
  get_filename_component(_dir ${_headerFile} PATH)
  list (APPEND PCAP_INCLUDE_DIRS ${_dir})
endforeach()
list (REMOVE_DUPLICATES PCAP_INCLUDE_DIRS)

include_directories(${PCAP_INCLUDE_DIRS})

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcap-visualizer ${PCAP_SOURCES})
target_link_libraries (pcap-visualizer ${PCL_LIBRARIES})
