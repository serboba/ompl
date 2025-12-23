# Install script for directory: /home/serboba/transferompl_ws/src/ompl_iso/demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/DubinsAirplane.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/DubinsAirplanePlot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/KinematicChainPathPlot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/OptimalPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/PlannerData.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/Point2DPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/RandomWalkPlanner.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/RigidBodyPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/RigidBodyPlanningWithControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/SpaceTimePlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/serboba/transferompl_ws/src/ompl_iso/demos/share/ompl/demos/StateSampling.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/BouncingBallPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/CForestCircleGridBenchmark.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/Diagonal.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/DubinsAirplane.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/GeometricCarPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/HybridSystemPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/HypercubeBenchmark.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/KinematicChainBenchmark.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/LTLWithTriangulation.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/OptimalPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/PinballPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/PlannerData.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/PlannerProgressProperties.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/Point2DPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/RigidBodyPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/RigidBodyPlanningWithControls.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/RigidBodyPlanningWithIK.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/SSTPinballPlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/SpaceTimePlanning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/StateSampling.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/ThunderLightning.cpp"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/TriangulationDemo.cpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/constraint"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/Koules"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/multilevel"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/PlanarManipulator"
    "/home/serboba/transferompl_ws/src/ompl_iso/demos/VFRRT"
    )
endif()

