cmake_minimum_required(VERSION 3.8)
project(vision_arm_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS nodes/cube_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ROS python style guide prefers single quotes; this codebase uses double
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_detector test/test_detector.py)
endif()

ament_package()
