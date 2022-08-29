cmake_minimum_required(VERSION 3.16)
project(Boids VERSION 1.0.0)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")

string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")

find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
find_package(ROOT COMPONENTS Core Hist HistPainter REQUIRED)

add_executable(BoidsExecutable main.cpp boids.cpp graphics.cpp SelbaWard/Ring.cpp stats.cpp)

target_link_libraries(BoidsExecutable PRIVATE sfml-graphics)
target_link_libraries(BoidsExecutable PRIVATE ROOT::Core ROOT::Hist ROOT::HistPainter)


if (BUILD_TESTING)

  add_executable(BoidsTest.t boids.test.cpp boids.cpp)
  add_executable(StatsTest.t stats.test.cpp boids.cpp stats.cpp)
  target_link_libraries(StatsTest.t PRIVATE ROOT::Core ROOT::Hist ROOT::HistPainter)

  add_test(NAME BoidsTest.t COMMAND BoidsTest.t)

endif()