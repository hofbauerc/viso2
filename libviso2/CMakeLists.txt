cmake_minimum_required(VERSION 3.5)
project(libviso2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

include_directories(
        include
        libviso2/src)

find_package(ament_cmake REQUIRED)

add_library(viso2 SHARED
        libviso2/src/filter.cpp
        libviso2/src/matcher.cpp
        libviso2/src/matrix.cpp
        libviso2/src/reconstruction.cpp
        libviso2/src/triangle.cpp
        libviso2/src/viso.cpp
        libviso2/src/viso_mono.cpp
        libviso2/src/viso_stereo.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(viso2)

install(
        DIRECTORY include/
        DESTINATION include)

install(TARGETS viso2
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
