cmake_minimum_required(VERSION 3.8)
project(ros2_autonomous_driving_application)

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

ament_package()

install(PROGRAMS
  scripts/uwb_publisher.py
  scripts/pure_pursuit_controller.py
  scripts/uwb_data_wifi_out.py
  scripts/imu.py
  scripts/uwb_publisher_dummy.py
  scripts/app_wifi_rx.py
  scripts/app_wifi_tx.py
  scripts/motor_cmd_vel_trx.py
  scripts/state_manager.py
  scripts/state_machine.py
  scripts/state_machine_for_cal.py
  scripts/cmd_send.py
  scripts/pure_pursuit_controller_v2.py
  scripts/motor_cmd_vel_trx_v2.py
  scripts/motor_cmd_vel_sim.py
  scripts/motor_cmd_vel_sim_1.py
  scripts/motor_cmd_vel_sim_2.py
  scripts/app_bridge.py
  scripts/imu_offset.py
  scripts/plot_calibration.py
  scripts/plot_ppc.py
  scripts/ppc.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)