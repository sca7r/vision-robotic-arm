cmake_minimum_required(VERSION 3.8)
project(vision_arm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config launch
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
  ament_add_pytest_test(test_configs test/test_configs.py)
endif()

ament_package()
