cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_CXX_FLAGS "-g -Wall")

PROJECT(HMPdetector)

# use gmr (in 'libs/GMM+GMR' subfolder)
include_directories("${PROJECT_SOURCE_DIR}/libs/GMM+GMR")
add_subdirectory(libs/GMM+GMR)
set(GMR_LIBS ${GMR_LIBS} GMM+GMR)

# use ChebyshevI (in 'libs/DspFilters' subfolder)
include_directories("${PROJECT_SOURCE_DIR}/libs/DspFilters")
add_subdirectory(libs/DspFilters)
set(FILTER_LIBS ${FILTER_LIBS} DspFilters)

# find Boost (thread, date-time, system) - required by SerialStream
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
if(USE_STATIC_BOOST)
    set(Boost_USE_STATIC_LIBS FALSE)
endif()
set(BOOST_LIBS ${BOOST_LIBS} thread date_time system)
find_package(Boost REQUIRED COMPONENTS ${BOOST_LIBS})

# find Threads - required by SerialStream
find_package(Threads REQUIRED)

ADD_EXECUTABLE(HMPdetector
  ./HMPdetector.cpp
  ./device.hpp ./MPU6050.hpp
  ./publisher.hpp ./logfile.hpp ./PEIS.hpp
  ./classifier.cpp ./classifier.hpp ./creator.cpp ./creator.hpp ./utils.cpp ./utils.hpp
  ./libs/SerialStream.cpp ./libs/SerialStream.h)

TARGET_LINK_LIBRARIES(HMPdetector ${GMR_LIBS} ${FILTER_LIBS} -larmadillo)
TARGET_LINK_LIBRARIES(HMPdetector -lpeiskernel_mt -lpeiskernel -lpthread)
TARGET_LINK_LIBRARIES(HMPdetector ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

INSTALL(
  TARGETS HMPdetector 
  RUNTIME DESTINATION /usr/local/bin
  LIBRARY DESTINATION /usr/local/lib
  ARCHIVE DESTINATION /usr/local/lib
)

