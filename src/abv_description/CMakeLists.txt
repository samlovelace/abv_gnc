cmake_minimum_required(VERSION 3.8)
project(abv_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Generate the URDF at build time
xacro_add_xacro_file(
    urdf/abv.urdf.xacro
    OUTPUT urdf/abv.urdf
)

install(
    DIRECTORY
        urdf
        meshes
        launch
        rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
