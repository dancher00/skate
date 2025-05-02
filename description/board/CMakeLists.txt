
cmake_minimum_required(VERSION 3.5)
project(board)

# Find required packages
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY meshes config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



# cmake_minimum_required(VERSION 2.8.3)

# project(board)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)


