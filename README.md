# Despliegue de modelos de Deep Learning en NVIDIA Jetson con MATLAB y ROS

Este repositorio documenta y distribuye los recursos necesarios para implementar **clasificadores** y **detectores de objetos** en una *NVIDIA Jetson Nano Developer Kit* mediante **MATLAB R2024a**, **GPU Coder**, **cuDNN**, **TensorRT** y **ROS Melodic**.  Se incluyen scripts de MATLAB para preparar y exportar redes, código en C++ para integrarlas como nodos ROS y ejemplos de inferencia tanto en el host como en la Jetson.

## Descripción general del proyecto

El objetivo es demostrar cómo llevar modelos de deep learning desarrollados en MATLAB a un dispositivo embebido Jetson y operarlos como servicios ROS en tiempo real.  Se abordan tres escenarios:

1. **Clasificación**: implementación de AlexNet, ResNet y VGG para reconocer objetos en imágenes de 227×227 px.  La librería se genera con **GPU Coder** y se despliega como un nodo ROS que procesa un flujo de cámara USB.
2. **Detección de objetos**: despliegue de un detector **YOLOv4** preentrenado para localizar y clasificar objetos (formato COCO).  Se proporcionan scripts para generar la biblioteca de inferencia y el nodo ROS correspondiente.
3. **Entrenamiento y personalización**: utilidades para convertir datasets al formato COCO, entrenar modelos personalizados con YOLOv4 y exportar detectores en MATLAB.

La carpeta `Original` contiene un laboratorio de referencia basado únicamente en AlexNet.  En ella se guía paso a paso la exportación de la red y su integración con ROS; la sección “Labs” del PDF explica estos pasos con detalle, incluyendo la configuración de `GPU Coder`, la preparación del entorno en la Jetson y la creación del nodo C++.

## Estructura de carpetas

El repositorio se divide en cuatro directorios principales, cada uno orientado a una tarea específica.  Las rutas y los archivos listados a continuación siguen el formato de árbol utilizado en la documentación de ROS (indicado en la siguiente sección).  Se omiten los archivos de dataset y de entrenamiento extensos para mantener la legibilidad.

### `Classification/`

Contiene todo lo necesario para generar y desplegar clasificadores de imágenes en la Jetson.  Su estructura es:

```

Classification/
├── JetsonCode/            # Código C++ del nodo ROS (clasificación)
│   ├── classifier.cpp
│   ├── classifier.hpp
│   ├── classifier\_usb.launch
│   ├── CMakeLists.txt
│   └── package.xml
├── Models/                # Redes preentrenadas en formato MAT
│   ├── alexnet.mat
│   ├── mnistCNN.mat
│   ├── resnet18.mat
│   ├── resnet50.mat
│   └── vgg16.mat
├── myClassifier.m         # Función de inferencia en MATLAB
├── myClassifierGPU.m      # Implementación preparada para GPU Coder
├── myClassifierGPU\_generation\_script.m  # Script para generar la .so
├── Samples/               # Imágenes de prueba
│   ├── car.jpg
│   ├── number\_0.jpg
│   ├── number\_2.jpg
│   ├── number\_3.jpg
│   ├── number\_7.jpg
│   ├── peppers.jpg
│   └── Person.jpg
├── saveNetworkToFile.m
├── testClassifier.m
└── visualizeROSClassifier.m

```

### `ObjectDetection/`

Incluye scripts y código de despliegue para el detector YOLOv4.

```

ObjectDetection/
├── JetsonCode/
│   ├── detector.cpp
│   ├── detector.hpp
│   ├── detector\_usb.launch
│   ├── CMakeLists.txt
│   └── package.xml
├── Models/
│   └── yoloV4\_coco.mat       # Detector YOLOv4 preentrenado (COCO)
├── myDetector.m
├── myDetectorGPU.m
├── myDetectorGPU\_generation\_script.m
├── Samples/
│   ├── car.jpg
│   └── Person.jpg
├── saveDetectorToFile.m
├── testDetector.m
└── visualizeROSDetector.m

```

### `Original/`

Laboratorio de referencia para AlexNet que ilustra la configuración completa del flujo de trabajo.  Se subdivide en:

```

Original/
├── AlexNetOriginal/
│   ├── alexnet.mat
│   ├── classNames.mat
│   ├── myAlexNet.m
│   ├── myAlexNetGPU.m
│   ├── myAlexNetGPU.prj
│   ├── myAlexNetGPU\_generation\_script.m
│   ├── peppers.jpg
│   ├── saveAlexNetToFile.m
│   ├── testAlexNet.m
│   └── visualizeROSData\_DEMO.m
└── JetsonCodeOriginal/
├── alexnet.cpp
├── alexnet.hpp
├── alexnet\_usb.launch
├── CMakeLists.txt
└── package.xml

```

### `TrainCustomModels/`

Herramientas para convertir datasets y entrenar detectores personalizados en MATLAB.

```

TrainCustomModels/
├── coco2table.m               # Convierte anotaciones COCO a tablas MATLAB
├── convertDataset.m           # Pasa datasets de imágenes a COCO
├── datasets/                  # Contiene subcarpetas para cada dataset
│   ├── cubes/
│   │   ├── train/             # Conjunto de entrenamiento
│   │   ├── valid/             # Conjunto de validación
│   │   └── test/              # Conjunto de prueba
│   └── drowsiness/
│       ├── train/
│       ├── valid/
│       └── test/
├── formatForYOLO.m
├── preprocessDataYOLOv4.m
├── runYOLOv4.m
├── trainBoxesYOLOv4.m
├── trainedDetectorYOLOv4.mat  # Detector YOLOv4 entrenado con datos propios
└── webcamPredict.m

```

## Estructura del workspace de ROS

Para ejecutar los modelos en la Jetson como nodos ROS, se utiliza un *workspace* Catkin estándar.  La guía de despliegue señala la siguiente organización:

```

/home/ucspjason/catkin\_ws/
└── src/
└── msra\_deployed\_nn/
├── CMakeLists.txt      # Build system del paquete
├── package.xml         # Metadatos del paquete ROS
├── launch/
│   └── alexnet\_usb.launch  # Launch file para lanzar el nodo con cámara USB
└── src/
├── alexnet.cpp      # Nodo C++ que invoca la biblioteca generada
└── alexnet.hpp      # Interfaz de la clase que maneja la inferencia

```

Cada directorio `JetsonCode` del proyecto contiene una estructura similar adaptada al modelo correspondiente (`classifier.cpp`/`detector.cpp`, etc.), con su propio `CMakeLists.txt`, `package.xml` y archivo `.launch`.

## Requisitos de hardware

Según la guía de laboratorio, el proyecto necesita tanto un PC anfitrión para el entrenamiento y la generación de código como una Jetson Nano para la inferencia:

| Dispositivo | Requisitos clave |
|---|---|
| **PC Anfitrión** | CPU multicore (Intel Core i7/i9 o AMD Ryzen 7/9), **≥16 GB** de RAM (recomendados 32 GB), GPU NVIDIA compatible con CUDA (p. ej. GeForce RTX 2060/3060 o superior) para acelerar la generación con GPU Coder, almacenamiento **SSD NVMe ≥ 500 GB** y conectividad Ethernet para transferencias rápidas. |
| **NVIDIA Jetson Nano** | Fuente de alimentación estable (5 V 3 A), disipador o ventilador para evitar sobrecalentamiento, **tarjeta microSD ≥64 GB** (clase 10; recomendable 128 GB) y periféricos (monitor HDMI, teclado, ratón).  Se recomienda conexión Ethernet o Wi‑Fi USB para ROS. |

## Requisitos de software

### En el PC host

Se utiliza **MATLAB R2024a** y las siguientes toolboxes:

- **Deep Learning Toolbox** y “Deep Learning Toolbox Model for AlexNet Network” para acceder a redes preentrenadas.
- **GPU Coder** y *MATLAB Coder Support Package for NVIDIA Jetson and DRIVE Platforms* para generar bibliotecas CUDA optimizadas para ARM 64 bits.
- **Image Processing Toolbox** y **Computer Vision Toolbox** para preprocesamiento de imágenes y funciones auxiliares.
- **ROS Toolbox** para establecer comunicación nativa con un *master* ROS.
- Compiladores y herramientas: **GCC/G++ ≥ 7.5**, **CMake ≥ 3.10** y **Make** para compilar los nodos C++ que generan GPU Coder.

### En la Jetson Nano

El dispositivo debe ejecutar **NVIDIA JetPack 4.6** (L4T R32.6.1) con los siguientes componentes:

- **CUDA 10.2** para ejecutar kernels de GPU.
- **cuDNN 8.2.1** y **TensorRT 8.2.1** para optimización de redes neuronales y generación de motores de inferencia.
- **OpenCV 4.x** con soporte CUDA (por defecto 4.1.1, actualizable a 4.8.0), **GStreamer 1.14.5** y **v4l-utils 1.14.2** para manejo de la cámara USB.
- **ROS Melodic Morenia** sobre Ubuntu 18.04 LTS, con los paquetes `ros-*-usb-cam` e `image_transport`.

## Uso y ejecución

1. **Preparar el modelo en MATLAB**: utilice los scripts `saveNetworkToFile.m`/`saveDetectorToFile.m` para exportar la red preentrenada a un archivo `.mat`.  Posteriormente, ejecute los scripts `myClassifierGPU_generation_script.m` o `myDetectorGPU_generation_script.m` para generar la biblioteca dinámica `.so` optimizada para Jetson, tal como se detalla en la guía.
2. **Configurar la Jetson**: instale JetPack 4.6 y configure `CUDA`, `cuDNN` y `TensorRT`.  Utilice `coder.checkGpuInstall` en MATLAB para verificar que la Jetson dispone de las librerías necesarias.
3. **Copiar la biblioteca a la Jetson**: transfiera el archivo `.so` generado al directorio apropiado (`alexnetCodeGen/codegen/dll/myModelGPU/`) dentro de la Jetson.
4. **Compilar el nodo ROS**: en la Jetson, cree un *workspace* catkin, copie el contenido de la carpeta `JetsonCode`, edite el `CMakeLists.txt` si es necesario para apuntar a la biblioteca generada y compile con `catkin_make`.
5. **Ejecutar el nodo**: lance `roscore` y el *launch file* (`alexnet_usb.launch` o `detector_usb.launch`) para iniciar el nodo.  Los nodos leen la cámara USB, recortan/escala las imágenes a 227×227 y publican el índice de clase o detecciones en tópicos ROS.
6. **Visualizar resultados**: desde MATLAB, utilice los scripts `visualizeROSClassifier.m` o `visualizeROSDetector.m` para subscribirse a los tópicos y mostrar la clasificación/detección en tiempo real.

## Personalización y entrenamiento de modelos

La carpeta `TrainCustomModels` provee utilidades para preparar datasets propios y entrenar detectores YOLOv4 personalizados.  El flujo típico consiste en:

1. **Preparar los datos**: colóquelos en `datasets/<nombre>/train`, `valid` y `test`, y ejecute `convertDataset.m` para convertir las anotaciones a formato COCO.
2. **Preprocesar para YOLOv4**: utilice `preprocessDataYOLOv4.m` y `formatForYOLO.m` para acondicionar los datos.
3. **Entrenar**: ejecute `trainBoxesYOLOv4.m` para entrenar el detector; el modelo resultante se almacena como `.mat` en la carpeta.
4. **Generar la biblioteca y nodo**: modifique `myDetectorGPU_generation_script.m` para cargar el detector entrenado y siga el mismo flujo de despliegue que con el modelo preentrenado.

## Créditos

Este proyecto fue desarrollado como parte de un laboratorio de la **Universidad Católica San Pablo** por Diego Banda.  Se tomó como fundamentos lo expuesto por **Jon Zeosky** y **Sebastian Castro** en su tutorial: [`Deep Learning with NVIDIA Jetson and ROS`](https://www.mathworks.com/matlabcentral/fileexchange/69366-deep-learning-with-nvidia-jetson-and-ros?s_eid=PSM_15028).

## Bibliografía:
1. [**NVIDIA Jetson Nano Developer Kit User Guide** – *Module and Thermal Design*: sección del manual oficial que explica cómo el disipador pasivo soporta hasta 10 W a 25 °C, y cómo añadir un ventilador PWM si se prevén cargas prolongadas para evitar thermal throttling.](https://developer.nvidia.com/downloads/jetson-nano-developer-kit-user-guide)
2. [**NVIDIA JetPack SDK 4.6 Release Notes** – *JetPack 4.6 Features*: página oficial que describe las versiones de CUDA (10.2), cuDNN (8.x), TensorRT (8.x) y VPI incluidas, así como mejoras de seguridad y soporte de OTAs para L4T 32.6.1.](https://developer.nvidia.com/embedded/jetpack-sdk-46)
3. [**GStreamer 1.14 Release Notes** – *1.14.5 Bug-Fix Release*: documento de Freedesktop.org que enumera correcciones de errores de la serie 1.14 y confirma que la versión 1.14.5 (29 de mayo 2019) es la última antes de pasar a 1.16.](https://gstreamer.freedesktop.org/releases/1.14/)
4. [**v4l-utils 1.14.2 Documentation** – *GitLab / LinTV*: repositorio que aloja la documentación de V4L2 y libdvbv5, esencial para entender parámetros como `pixel_format` y `framerate` al usar la webcam en Jetson.](https://gitlab.collabora.com/adalessandro/v4l-utils/-/tree/v4l-utils-1.14.2)
5. [**ROS Wiki** – *Ubuntu install of ROS Melodic*: instrucciones oficiales para configurar repositorios, claves, dependencias y entorno en Ubuntu 18.04, requisito previo para instalar `ros-melodic-usb-cam` e `image_transport`.](https://wiki.ros.org/melodic/Installation/Ubuntu)
6. [**MathWorks File Exchange** – *Deep Learning Toolbox Model for AlexNet Network*: paquete oficial que provee un objeto `SeriesNetwork` entrenado sobre ImageNet, con 23 capas y capaz de reconocer 1 000 categorías, listo para exportar a `.mat`.](https://www.mathworks.com/matlabcentral/fileexchange/59133-deep-learning-toolbox-model-for-alexnet-network)
7. [**MathWorks Documentation** – *MATLAB Coder Support Package for NVIDIA Jetson and NVIDIA DRIVE Platforms*: descripción del paquete que automatiza la generación remota y despliegue de código MATLAB/Simulink en plataformas NVIDIA, incluyendo configuración SSH y manejo de binarios `.so`.](https://www.mathworks.com/help/coder/nvidia.html)
8. [**Deep Learning with MATLAB, NVIDIA Jetson, and ROS**: vídeo de MathWorks en el que Jon Zeosky y Sebastian Castro muestran cómo partir de una red neuronal preentrenada en MATLAB para generar, con GPU Coder, una biblioteca CUDA independiente, desplegarla en una plataforma NVIDIA Jetson y, finalmente, empaquetarla como nodo ROS en C++ para integrarla con otros nodos de la red.](https://www.mathworks.com/videos/matlab-and-simulink-robotics-arena-deep-learning-with-nvidia-jetson-and-ros--1542015526909.html)
9. [**Deep Learning with NVIDIA Jetson and ROS**: paquete de ejemplo en MATLAB Central File Exchange que contiene los archivos para desplegar en hardware NVIDIA Jetson una red neuronal preentrenada desde MATLAB, y utilizar la biblioteca generada dentro de un nodo ROS escrito en C++. Incluye README con instrucciones de configuración y requisitos de Toolbox.](https://www.mathworks.com/matlabcentral/fileexchange/69366-deep-learning-with-nvidia-jetson-and-ros?s_eid=PSM_15028)
10. [**MathWorks Miniseries | April 29, 2024 | Object Detection with ROS 2 and Jetson Nano**: episodio en directo de la serie *Miniseries* donde se enseña a usar el Image Labeler App de MATLAB para etiquetar datos, entrenar una red de detección de objetos y, mediante la generación de código de MATLAB, desplegarla como nodo ROS 2 en un Jetson Nano (u Orin Nano), cubriendo el flujo completo de etiqueta→entrenamiento→generación→despliegue.](https://www.youtube.com/watch?v=FHSVW5-W5ew)
11. [**Deploy YOLOv2 to an NVIDIA Jetson**: vídeo en el que Connell D’Souza y Neha Goel explican cómo usar GPU Coder para generar código CUDA de la red YOLOv2 diseñada en MATLAB, configurar el objetivo Jetson y desplegar los binarios resultantes en la GPU embebida, permitiendo realizar inferencia en tiempo real directamente en el dispositivo.](https://www.youtube.com/watch?v=fD-PKiqYNKo)