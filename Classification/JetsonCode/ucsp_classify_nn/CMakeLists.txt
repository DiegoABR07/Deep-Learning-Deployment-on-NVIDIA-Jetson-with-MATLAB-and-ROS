###############################################################################
# 0.  Versión mínima de CMake y nombre del paquete
###############################################################################
cmake_minimum_required(VERSION 3.10)      # 3.10+ funciona bien en Jetson
project(ucsp_classify_nn LANGUAGES CXX)

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
# 2.  Rutas de la red exportada por GPU Coder (actualizado para myClassifierGPU)
###############################################################################
# Original (AlexNet):
# set(CODEGEN_FCN_NAME myAlexNetGPU)
# set(CODEGEN_DIR      /home/ucspjason/alexnetCodeGen)
# add_library(myAlexNetGPU SHARED IMPORTED)
# set_target_properties(myAlexNetGPU PROPERTIES
#   IMPORTED_LOCATION
#     /home/ucspjason/alexnetCodeGen/codegen/dll/myAlexNetGPU/myAlexNetGPU.so)

# Actualizado (Classifier genérico):
set(CODEGEN_FCN_NAME myClassifierGPU)
# Carpeta raíz donde está el código generado por GPU Coder
# Cambia esta ruta a donde tengas tu carpeta de codegen de myClassifierGPU
set(CODEGEN_DIR      /home/ucspjason/myClassifierGPU)

# Registramos la biblioteca compartida generada por GPU Coder
add_library(myClassifierGPU SHARED IMPORTED)
set_target_properties(myClassifierGPU PROPERTIES
  IMPORTED_LOCATION
    ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/${CODEGEN_FCN_NAME}.so
)

# Definimos la ruta absoluta a la biblioteca para instalarla más abajo
set(CODEGEN_LIB
    ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/${CODEGEN_FCN_NAME}.so)

###############################################################################
# 3.  CUDA (solo para cabeceras y librerías estándar)
###############################################################################
find_package(CUDA REQUIRED)   # rellena CUDA_INCLUDE_DIRS y CUDA_LIBRARIES

###############################################################################
# 4.  Cabeceras
###############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  # Ruta a la carpeta donde esté miClassifierGPU.h
  ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}
  ${CUDA_INCLUDE_DIRS}
)

###############################################################################
# 5.  Ejecutable del nodo
###############################################################################
# Cambiado de "alexnet" a "classifier"
add_executable(classifier src/classifier.cpp)

target_link_libraries(classifier
  myClassifierGPU            # Ya no es AlexNet, sino el clasificador genérico
  ${CUDA_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libcudnn.so
  ${catkin_LIBRARIES}
)

# Dependencia para que se genere el mensaje antes de compilar el nodo (si aplica)
add_dependencies(classifier sensor_msgs_generate_messages_cpp)

###############################################################################
# 6.  Exportación e instalación ROS
###############################################################################
# catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport)

# Copiamos la .so a devel/lib para que ROS la encuentre en tiempo de ejecución
install(FILES ${CODEGEN_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# (opcional) instala también los ficheros de pesos / auxiliares si los necesitas
install(DIRECTORY ${CODEGEN_DIR}/codegen/dll/${CODEGEN_FCN_NAME}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.bin")
