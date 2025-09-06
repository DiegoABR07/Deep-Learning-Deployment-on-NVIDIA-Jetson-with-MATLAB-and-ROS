###############################################################################
# 0.  Versión mínima de CMake y nombre del paquete
###############################################################################
cmake_minimum_required(VERSION 3.10)
project(ucsp_detect_nn LANGUAGES CXX)

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
# 2.  Rutas de la red exportada por GPU Coder (detector genérico)
###############################################################################
# Nombre de la función generada por GPU Coder
set(CODEGEN_FCN_NAME myDetectorGPU)
# Carpeta raíz donde está el código generado para myDetectorGPU.
# AJUSTA esta ruta a la ubicación real de tu codegen de myDetectorGPU:
set(CODEGEN_DIR      /home/ucspjason/myDetectorGPU)

# Insertar la biblioteca compartida (.so) generada
add_library(myDetectorGPU SHARED IMPORTED)
set_target_properties(myDetectorGPU PROPERTIES
  IMPORTED_LOCATION
    ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/${CODEGEN_FCN_NAME}.so
)

# Ruta absoluta a la biblioteca, para instalarla luego
set(CODEGEN_LIB
    ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/${CODEGEN_FCN_NAME}.so)

###############################################################################
# 3.  CUDA (cabeceras y bibliotecas estándar)
###############################################################################
find_package(CUDA REQUIRED)   # define CUDA_INCLUDE_DIRS y CUDA_LIBRARIES

###############################################################################
# 4.  Cabeceras
###############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}  # Para myDetectorGPU.h
  ${CUDA_INCLUDE_DIRS}
)

###############################################################################
# 5.  Ejecutable del nodo
###############################################################################
add_executable(detector_node src/detector.cpp)

target_link_libraries(detector_node
  myDetectorGPU             # Biblioteca importada de GPU Coder
  ${CUDA_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libcudnn.so
  ${catkin_LIBRARIES}
)

add_dependencies(detector_node sensor_msgs_generate_messages_cpp)

###############################################################################
# 6.  Exportación e instalación ROS
###############################################################################
# catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport)

# Copiamos la .so a devel/lib para que ROS la encuentre en tiempo de ejecución
install(FILES ${CODEGEN_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# (opcional) instala ficheros auxiliares (.bin, pesos, etc.)
install(DIRECTORY ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bin")
