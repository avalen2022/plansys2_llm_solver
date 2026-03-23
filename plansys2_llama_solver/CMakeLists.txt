cmake_minimum_required(VERSION 3.5)

project(plansys2_llama_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_solver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_llama_solver/llama_solver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  plansys2_solver::plansys2_solver
  PRIVATE
  pluginlib::pluginlib
  rclcpp::rclcpp
  nlohmann_json::nlohmann_json
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(plansys2_solver plansys2_llama_solver_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plansys2_llama_solver_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  plansys2_solver
  pluginlib
)

ament_package()
