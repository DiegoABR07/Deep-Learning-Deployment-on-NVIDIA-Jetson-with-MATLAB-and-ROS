# Ejemplo Original: AlexNet en Jetson Nano

Esta carpeta contiene el ejemplo original de la guía “Despliegue de AlexNet en una NVIDIA Jetson Nano con MATLAB R2024a, GPU Coder y ROS Melodic”. Incluye los scripts y datos de MATLAB necesarios para generar una biblioteca optimizada de AlexNet, probar el modelo localmente y visualizar la inferencia sobre ROS. Además, contiene la estructura básica del paquete ROS escrito en C++ para desplegar la red en la Jetson (archivos .cpp, .hpp y .launch), cuya explicación se detalla a continuación.

## 📂 Estructura del directorio

```text
Original/
├── AlexNetOriginal/
│   ├── alexnet.mat              # Red AlexNet preentrenada
│   ├── classNames.mat           # Nombres de las clases de ImageNet
│   ├── myAlexNet.m              # Inferencia en CPU
│   ├── myAlexNetGPU.m           # Función compatible con GPU Coder
│   ├── myAlexNetGPU.prj         # Proyecto GPU Coder preconfigurado
│   ├── myAlexNetGPU_generation_script.m  # Script para generar la .so
│   ├── peppers.jpg              # Imagen de prueba
│   ├── project_structure.text   # Descripción de carpetas
│   ├── saveAlexNetToFile.m      # Guarda la red AlexNet en .mat
│   ├── testAlexNet.m            # Prueba inferencia en MATLAB
│   └── visualizeROSData_DEMO.m  # Demo de visualización vía ROS
└── JetsonCodeOriginal/
    ├── alexnet.cpp
    ├── alexnet.hpp
    ├── alexnet_usb.launch
    ├── CMakeLists.txt
    └── package.xml
````

## 🧠 Flujos de MATLAB

### `saveAlexNetToFile.m`

* Guarda la red AlexNet preentrenada en un archivo `.mat`.
* La función usa `net = alexnet; save('alexnet.mat','net');` para almacenar el objeto `SeriesNetwork`.
* El archivo `alexnet.mat` contiene únicamente la red; los nombres de clase se almacenan en `classNames.mat`.

### `myAlexNet.m`

* Función de inferencia en **CPU**.
* Carga la red `alexnet.mat` y la lista de clases (`classNames.mat`) de forma persistente para evitar recargas.
* Preprocesa la imagen a 227×227 px RGB (cambia canales, escala a \[0,1]) y llama a `predict`.
* Devuelve el índice de clase con mayor probabilidad y el nombre de la clase.

### `myAlexNetGPU.m`

* Versión compatible con **GPU Coder**.
* Utiliza `coder.loadDeepLearningNetwork('alexnet.mat')` para cargar la red una sola vez.
* Acepta una imagen `uint8(227×227×3)` y devuelve el índice de la clase máxima usando `predict`.
* No requiere lista de clases; sólo genera un entero.

### `myAlexNetGPU_generation_script.m`

* Script para generar la biblioteca compartida `myAlexNetGPU.so`.
* Define una configuración `coder.gpuConfig('lib','ecoder',true)` con destino Jetson, habilitando la generación de informes y fijando la plataforma como ARM 64-bit.
* Declara el tipo de entrada como `uint8(227×227×3)` y llama a:

```matlab
codegen -config cfg -o myAlexNetGPU myAlexNetGPU -args {imType} -nargout 1
```

* El resultado es una librería (`myAlexNetGPU.so`), archivos `.h/.cpp` y un informe HTML.

### `myAlexNetGPU.prj`

* Archivo de proyecto de GPU Coder preconfigurado para AlexNet.
* Permite generar la biblioteca a través de la interfaz gráfica en MATLAB (opcional frente al script).

### `testAlexNet.m`

* Script de prueba para validar la inferencia en MATLAB:

  1. Carga `alexnet.mat` y `classNames.mat`.
  2. Lee una imagen (`peppers.jpg` u otra).
  3. Redimensiona la imagen a 227×227 px.
  4. Ejecuta `myAlexNet(im)` o `myAlexNetGPU(im)` dependiendo del modo (CPU/GPU).
  5. Muestra la imagen y escribe el nombre de la clase predicha.

### `visualizeROSData_DEMO.m`

* Demo para **visualizar los resultados** publicados por el nodo ROS en la Jetson:

  * Conecta a ROS (función `rosinit`) en la Jetson.
  * Se suscribe a `/network_view` (imagen recortada) y `/network_out` (índice de clase).
  * Carga `classNames.mat` para mapear índices a nombres.
  * Dibuja la imagen y anota el nombre de la clase predicha.
* Permite monitorizar el desempeño del modelo en tiempo real desde el PC host.

### `classNames.mat` y `alexnet.mat`

* `classNames.mat` contiene un arreglo de 1 000 nombres de clase, correspondientes a la base de datos ImageNet.
* `alexnet.mat` almacena la red AlexNet preentrenada.

---

## 🚀 Estructura y Archivos de ROS

La carpeta `JetsonCodeOriginal/` contiene el paquete ROS `msra_deployed_nn`, formado por los siguientes archivos:

```text
JetsonCodeOriginal/
├── alexnet.cpp            # Punto de entrada del nodo
├── alexnet.hpp            # Clase GpuNetPub con la lógica de inferencia
├── alexnet_usb.launch     # Archivo de lanzamiento para Jetson
├── CMakeLists.txt         # Build system del paquete
└── package.xml            # Declaración de dependencias y metadatos
```

### Estructura del paquete y CMake

* **package.xml:** define el paquete `msra_deployed_nn` (versión 1.0.0) y declara como dependencias los paquetes `roscpp`, `std_msgs`, `sensor_msgs`, `image_transport` y `usb_cam`. Estos son necesarios para manejar suscripciones y publicaciones de imágenes, mensajes tipo `UInt32` y el driver de cámara.
* **CMakeLists.txt:** fija el nombre del proyecto a `msra_deployed_nn` y especifica la ruta al código generado (`CODEGEN_DIR`) y la función de entrada (`CODEGEN_FCN_NAME = myAlexNetGPU`). Importa la biblioteca `myAlexNetGPU.so`, incluye las cabeceras de la biblioteca generada y las de CUDA, y crea el ejecutable `alexnet` a partir de `src/alexnet.cpp`. Vincula el ejecutable con la biblioteca generada, las librerías de CUDA, cuDNN y las dependencias de ROS, e instala la `.so` en el destino de catkin.

### Comportamiento del nodo

* **alexnet.cpp:** inicializa ROS, crea un objeto `GpuNetPub` que publicará en los tópicos `network_out` (índice de clase) y `network_view` (imagen recortada) y obtiene el parámetro `input_topic` (por defecto `/usb_cam/image_raw`) desde el espacio de nombres privado (`~`). A continuación se suscribe a dicho tópico y entra en `ros::spin()`.
* **alexnet.hpp:** la clase `GpuNetPub` implementa `msgCallback()`, que realiza:

  1. **Validación y recorte:** comprueba que la imagen entrante sea al menos de 227×227 px; si es más pequeña, emite una advertencia. Calcula un recorte centrado de 227×227 px.
  2. **Conversión de datos:** copia los valores del recorte a dos buffers: uno en formato **column-major** (orden `H×W×C`) requerido por GPU Coder y otro en **row-major** para publicar la subimagen. El recorte conserva los 3 canales (`rgb`).
  3. **Inferencia:** llama a `myAlexNetGPU()` con el arreglo column-major para obtener el índice de clase (tipo `float`), que convierte a `uint32_t`.
  4. **Publicación:** envía el índice de clase en el tópico `/network_out` (tipo `std_msgs::UInt32`) y la subimagen recortada (formato `rgb8`) en el tópico `/network_view` (`sensor_msgs::Image`).
  5. **Mensajes de estado:** imprime el índice de clase a la consola a una frecuencia reducida y advierte si el frame es demasiado pequeño o no es RGB.

### Launch file

El archivo `alexnet_usb.launch` realiza el despliegue del nodo en la Jetson:

* Ejecuta el nodo `alexnet` del paquete `msra_deployed_nn` y permite configurar el parámetro `input_topic` a través de `<param name="input_topic" value="..."/>`.
* Lanza también el nodo `usb_cam_node` (del paquete `usb_cam`) con una cámara USB: resolución 640×360 px, 30 FPS y formato `yuyv`, desactivando el balance de blancos y el enfoque automáticos para obtener frames consistentes.
* Este flujo simplifica la reproducción del ejemplo: sólo necesitas conectar una cámara a la Jetson y lanzar el archivo `.launch` para iniciar tanto la captura como la inferencia.

### Flujo de despliegue en Jetson

Para desplegar el modelo AlexNet en la Jetson Nano:

1. **Copiar la biblioteca generada** (`myAlexNetGPU.so`) y las cabeceras asociadas al directorio configurado en `CMakeLists.txt` (`CODEGEN_DIR`).

2. **Crear el workspace catkin** en la Jetson (`~/catkin_ws/src`) y copiar allí la carpeta `JetsonCodeOriginal`.

3. **Compilar e instalar** el paquete:

   ```bash
   cd ~/catkin_ws
   catkin_make -DCMAKE_BUILD_TYPE=Release
   source devel/setup.bash
   ```

4. **Lanzar la inferencia**:

   ```bash
   roslaunch msra_deployed_nn alexnet_usb.launch
   ```

5. **Visualizar los resultados** en el PC host ejecutando `visualizeROSData_DEMO.m`, que se suscribe a `/network_view` y `/network_out` y muestra la imagen recortada junto con el nombre de la clase predicha.

---

## 🧪 Flujo sugerido

1. **Guardar la red**: Ejecuta `saveAlexNetToFile.m` para crear `alexnet.mat`.
2. **Generar la biblioteca CUDA**: Modifica `myAlexNetGPU_generation_script.m` si es necesario y ejecútalo para producir `myAlexNetGPU.so`.
3. **Desplegar en Jetson**: Copia la biblioteca y compila el nodo C++ siguiendo las instrucciones anteriores.
4. **Probar localmente**: Usa `testAlexNet.m` para asegurarte de que la red funciona correctamente.
5. **Visualizar en ROS**: Lanza el nodo en la Jetson y corre `visualizeROSData_DEMO.m` para ver la clasificación en tiempo real.