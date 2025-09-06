###############################################################################
# 0.  Versión mínima de CMake y nombre del paquete
###############################################################################
cmake_minimum_required(VERSION 3.10)      # 3.10+ funciona bien en Jetson
project(msra_deployed_nn LANGUAGES CXX)

###############################################################################
# 1.  Paquetes ROS que usa este nodo
###############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport)

###############################################################################
# 2.  Rutas de la red exportada por GPU Coder
###############################################################################
set(CODEGEN_FCN_NAME myAlexNetGPU)
set(CODEGEN_DIR      /home/ucspjason/alexnetCodeGen)
# set(CODEGEN_LIB /home/ucspjason/alexnetCodeGen/codegen/dll/myAlexNetGPU/myAlexNetGPU.so)
add_library(myAlexNetGPU SHARED IMPORTED)
set_target_properties(myAlexNetGPU PROPERTIES
  IMPORTED_LOCATION
    /home/ucspjason/alexnetCodeGen/codegen/dll/myAlexNetGPU/myAlexNetGPU.so)

###############################################################################
# 3.  CUDA (solo para cabeceras y librerías estándar)
###############################################################################
find_package(CUDA REQUIRED)   # rellena CUDA_INCLUDE_DIRS y CUDA_LIBRARIES

###############################################################################
# 4.  Cabeceras
###############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}   # myAlexNetGPU.h
  ${CUDA_INCLUDE_DIRS}
)

###############################################################################
# 5.  Ejecutable del nodo
###############################################################################
add_executable(alexnet src/alexnet.cpp)

target_link_libraries(alexnet
  myAlexNetGPU            # ← ya no se forma -l, se pasa la ruta exacta
  ${CUDA_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libcudnn.so
  ${catkin_LIBRARIES})

add_dependencies(alexnet sensor_msgs_generate_messages_cpp)

###############################################################################
# 6.  Exportación e instalación ROS
###############################################################################
# catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport)

# Copiamos la .so a devel/lib para que ROS la encuentre en tiempo de ejecución
install(FILES ${CODEGEN_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# (opcional) instala también los ficheros de pesos / auxiliares si los necesitas
install(DIRECTORY ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bin")
