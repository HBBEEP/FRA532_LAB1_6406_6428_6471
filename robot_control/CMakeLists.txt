cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY 
    scripts
  DESTINATION 
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ik.py
    scripts/purepursuit_example.py
    scripts/simple_controller_example.py
    scripts/vff_avoidance_ex.py
    scripts/vff_avoidance_sol.py
    scripts/pp_w_vff.py
    scripts/velocity_controller.py
    scripts/wheel_controller.py
    scripts/test_map.py
    scripts/global_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
