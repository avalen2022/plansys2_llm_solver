cmake_minimum_required(VERSION 3.5)
project(plansys2_solver_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Solver.msg"
  "msg/SolverArray.msg"
  "srv/GetSolve.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
