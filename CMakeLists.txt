cmake_minimum_required(VERSION 3.0.0)

project(HaxBallAgentGroup-5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -ffast-math")

find_package(Qt5 COMPONENTS Widgets Gui REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
    HaxBallEnv/
    src/
    HaxBallEnv/include
    HaxBallEnv/include/Eigen)

set(SRC_FILES
    main.cpp
    src/AgentDP.cpp
    src/AgentTDSituative.cpp
    src/Features.cpp
    src/Rewards.cpp
    src/AgentTDmaster.cpp
    HaxBallEnv/src/BaseAgent.cpp
    HaxBallEnv/src/RandomAgent.cpp
    HaxBallEnv/src/HaxBall.cpp
    HaxBallEnv/src/HaxBallGui.cpp)

set(MOC_FILES
    HaxBallEnv/include/HaxBall.h
    HaxBallEnv/include/HaxBallGui.h)

qt5_wrap_cpp(SRC_FILES ${MOC_FILES})

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Gui OpenMP::OpenMP_CXX)
