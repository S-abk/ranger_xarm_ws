cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch meshes
  DESTINATION share/${PROJECT_NAME}
)
# CC-BY 4.0 text for the printable mount meshes, installed beside them.
install(FILES LICENSE-CC-BY-4.0
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The committed urdf/mass_properties.generated.xacro is what config/mass_properties.yaml
  # gives, and the expanded description's masses match the targets. Starts nothing.
  ament_add_pytest_test(test_mass_properties test/test_mass_properties.py
    TIMEOUT 180)
endif()

ament_package()
