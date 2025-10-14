cmake_minimum_required(VERSION 3.10)
project(voice_control)

# ใช้ ament_cmake และ ament_cmake_python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -----------------------------------------------------
# ติดตั้ง Python package (ต้องมี __init__.py ในโฟลเดอร์ voice_control/)
# -----------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------------------------------------
# ติดตั้ง Python scripts ที่ใช้เป็น node
# -----------------------------------------------------
install(PROGRAMS
  launch/voice_processor.py
  launch/voice_speech.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------
# ติดตั้งไฟล์ประกอบอื่น ๆ เช่น config หรือคำสั่ง
# -----------------------------------------------------
install(FILES
  commands.json
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------
# Export dependencies (ให้ package อื่นหาเจอ)
# -----------------------------------------------------
ament_export_dependencies(
  rclpy
  std_msgs
  sensor_msgs
)

ament_package()
