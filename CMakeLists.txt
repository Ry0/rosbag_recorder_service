cmake_minimum_required(VERSION 3.8)
project(rosbag_recorder_service)

# デフォルトのC++14コンパイルとコンパイルオプション
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係の検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# サービス定義の生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RecordStart.srv"
  "srv/RecordStop.srv"
  "srv/GetStatus.srv"
  DEPENDENCIES
)

# インクルードディレクトリの設定
include_directories(include)

# ビルドするライブラリとソースファイル
add_library(${PROJECT_NAME}_lib
  src/rosbag_recorder_service.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
)

# 自身のサービス定義に対する依存関係の設定
rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 実行可能ファイルの作成
add_executable(${PROJECT_NAME}_node src/rosbag_recorder_service_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

# インストール設定
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# テスト設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# パッケージエクスポート
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
)

ament_package()