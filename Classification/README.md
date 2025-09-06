# Clasificación de imágenes

Esta carpeta contiene todos los scripts de MATLAB y archivos ROS necesarios para **clasificar imágenes** en tiempo real utilizando redes neuronales profundas desplegadas en una *NVIDIA Jetson Nano*.

El objetivo es cargar un modelo de clasificación entrenado (AlexNet, ResNet, VGG, etc.), generar una biblioteca CUDA optimizada con **GPU Coder**, integrarla en un nodo ROS en C++ y recibir resultados de clasificación a través de tópicos ROS. El nodo actúa como puente entre la cámara USB y la biblioteca generada, recortando y reordenando cada fotograma, ejecutando la inferencia y publicando tanto la subimagen preprocesada como el índice de clase.

---

## 📂 Estructura del directorio

```

Classification/
├── JetsonCode/                # Código C++ del nodo ROS de clasificación
│   └── ucsp_classify_nn/
│       ├── CMakeLists.txt        # Sistema de build para el paquete ROS
│       ├── package.xml           # Metadatos y dependencias del paquete
│       ├── launch/
│       │   └── classifier_usb.launch  # Archivo de lanzamiento para cámara USB
│       └── src/
│           ├── classifier.cpp    # Implementación del nodo de clasificación
│           └── classifier.hpp    # Declaración de la clase ClassifierNode
├── Models/                    # Redes preentrenadas en formato .mat
│   ├── alexnet.mat
│   ├── mnistCNN.mat
│   ├── resnet18.mat
│   ├── resnet50.mat
│   └── vgg16.mat
├── myClassifier.m             # Clasificador genérico (CPU)
├── myClassifierGPU.m          # Función preparada para GPU Coder
├── myClassifierGPU_generation_script.m  # Script de generación de la biblioteca
├── Samples/                   # Imágenes de ejemplo
│   ├── car.jpg
│   ├── number_0.jpg
│   └── …
├── saveNetworkToFile.m        # Guarda cualquier red en un .mat
├── testClassifier.m           # Prueba de inferencia en CPU
├── visualizeROScamera.m       # Demo para visualizar la cámara ROS
└── visualizeROSClassifier.m   # Visualiza imagen + clase desde ROS

````

---

## 🧠 Flujos de MATLAB

A continuación se describen brevemente los scripts principales incluidos en este directorio:

### `saveNetworkToFile.m`

- Función genérica que guarda un objeto de red (`SeriesNetwork`, `DAGNetwork` o `dlnetwork`) en un archivo `.mat` bajo el nombre de variable `net`.  
- Uso:  

  ```matlab
  net = resnet18;
  saveNetworkToFile(net, 'resnet18.mat');
  ```

*No guarda las clases por separado; estas se pueden recuperar mediante `net.Layers(end).Classes`.*

### `myClassifier.m`

* Clasificador genérico que funciona en **CPU**.
* Recibe una imagen arbitraria y un archivo `.mat` que contiene la red entrenada.
* Carga de forma persistente la red y calcula automáticamente el tamaño de entrada y los nombres de clase.
* Preprocesa la imagen: ajusta el número de canales (RGB/gris), redimensiona al tamaño requerido y puede normalizar a single \[0,1].
* Ejecuta `predict` sobre la red y devuelve el índice de la clase con mayor probabilidad y, si existen, los nombres de clase.

### `myClassifierGPU.m`

* Versión preparada para **GPU Coder**.
* Espera imágenes de tamaño constante y el nombre del modelo como argumentos constantes para facilitar la compilación.
* Utiliza `coder.loadDeepLearningNetwork` para cargar la red en el dispositivo de destino y realiza operaciones similares a `myClassifier` (preprocesamiento de canales, redimensionado y predicción).

### `myClassifierGPU_generation_script.m`

* Script que genera la biblioteca CUDA para Jetson Nano.

* Limpia el entorno (`clc; clear; close all`) y define el archivo del modelo (`modelFile`).

* Lee el `.mat` para determinar el `inputSize`, que luego se utiliza para definir el tipo de entrada (`imType`).

* Configura el objeto `cfg = coder.gpuConfig('dll','ecoder',true)` para un objetivo **ARM 64-bit** con Jetson, habilitando la generación de informes de código.

* Construye el arreglo `ARGS` con tipos constantes y llama a:

  ```matlab
  codegen -config cfg -o myClassifierGPU ...
          myClassifierGPU -args ARGS -nargout 1
  ```

* El resultado es una biblioteca compartida `myClassifierGPU.so` lista para enlazarse en Jetson.

### `testClassifier.m`

* Script de ejemplo para probar un modelo en **CPU**.
* Carga una imagen de prueba (`Samples/peppers.jpg` por defecto) y un modelo (`Models/vgg16.mat` por defecto).
* Llama a `myClassifier` para obtener el índice y nombre de la clase, muestra la imagen y escribe el resultado en consola.

### `visualizeROScamera.m`

* Demo sencilla para **probar la cámara ROS**.
* Conecta a un maestro ROS en la Jetson (IP configurable), se suscribe al tópico `/network_view` y muestra en bucle las imágenes recibidas con `imshow`.

### `visualizeROSClassifier.m`

* Visualiza en tiempo real el flujo de cámara y la predicción de clase enviada desde Jetson.
* Conecta a ROS, carga el archivo `.mat` para conocer las clases, se suscribe a `/network_view` (imágenes) y a `/network_out` (índices de clase).
* Para cada mensaje:

  * Convierte la imagen (`sensor_msgs/Image`) a formato MATLAB con `readImage`.
  * Obtiene el índice de clase, lo valida y recupera el nombre correspondiente.
  * Muestra la imagen y el nombre de la clase; si no hay mensaje, dibuja un placeholder con el tamaño requerido.

---

## 🤖 Despliegue y nodo ROS

### Estructura del paquete ROS

El código C++ que corre en la Jetson se organiza dentro del paquete `ucsp_classify_nn`. La estructura básica es:

```
ucsp_classify_nn/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── classifier_usb.launch
└── src/
    ├── classifier.cpp
    └── classifier.hpp
```

#### `classifier.hpp`

* Declara la clase `ClassifierNode`, que encapsula todo el comportamiento del nodo.
* Almacena:

  * Publicadores `data_pub_` (para el índice de clase) e `img_pub_` (para la subimagen).
  * Parámetros `SUB_H`, `SUB_W` y `CHANNELS` que determinan el tamaño y el número de canales de la subimagen de entrada a la red.
  * El último `encoding` recibido para fines de depuración.
* Incluye la cabecera generada por GPU Coder (`myClassifierGPU.h`), que debe encontrarse en el `include` path.

#### `classifier.cpp`

Implementa el nodo de clasificación en C++:

1. **Constructor** (`ClassifierNode`)

   * Registra los publicadores para los tópicos de salida (`/network_out`) y visualización (`/network_view`).
   * Informa los valores de `SUB_H`, `SUB_W` y `CHANNELS` a través de `ROS_INFO`.

2. **Callback de imagen (`msgCallback`)**

   * **Detección de encoding**: Comprueba el encoding de la imagen entrante (admite `rgb8`, `bgr8` y `mono8`) y emite advertencias si no está soportado.
   * **Verificación de tamaño**: Asegura que la imagen tenga dimensiones suficientes para el recorte; de lo contrario, emite un `ROS_WARN_THROTTLE`.
   * **Reserva de buffers**:

     * `buffer_raw`: contiene los datos RGB en formato **planar** (R plane, G plane, B plane), tal como los espera `myClassifierGPU`.
     * `img_msg_data`: almacena la imagen recortada en formato **intercalado** (`rgb8`) para publicarla en ROS.
   * **Cálculo de offsets**: Calcula los offsets de altura y anchura para realizar un recorte **centrado**.
   * **Copia y mapeo de canales**: Para cada píxel recortado:

     * Convierte de `bgr8` a `rgb8` si es necesario o repite el valor para imágenes en escala de grises.
     * Guarda los datos en formato planar para la red y en formato intercalado para ROS.
   * **Depuración opcional**: Calcula estadísticos (`min`, `max`, `mean`) del buffer para comprobar que la imagen tiene valores variados.
   * **Invocación de la red**: Llama a `myClassifierGPU(buffer_raw)` y convierte el resultado a `uint32_t`.
   * **Publicación**:

     * Envía el índice de clase a `/network_out` (`std_msgs::UInt32`).
     * Construye y publica la subimagen recortada en formato `rgb8` a `/network_view` (`sensor_msgs::Image`), conservando el encabezado original para sincronización.

3. **Función `main`**

   * Inicializa ROS y define un `NodeHandle` global y uno privado (`pnh("~")`).
   * Define parámetros con valores por defecto:

     * `input_topic` (tópico de la cámara, por defecto `/usb_cam/image_raw`)
     * `sub_h` y `sub_w` (tamaño del recorte, por defecto 224×224, ajustable según tu modelo; por ejemplo, 227×227 para AlexNet)
     * `channels` (generalmente 3)
   * Carga los parámetros desde el servidor de parámetros (pueden modificarse en el archivo `.launch`).
   * Crea una instancia de `ClassifierNode` y se suscribe al tópico de entrada.
   * Entra en el bucle de eventos con `ros::spin()`.

### `CMakeLists.txt`

El archivo de build especifica:

* **Dependencias**: Usa `catkin` con los paquetes `roscpp`, `std_msgs`, `sensor_msgs` e `image_transport`.
* **Biblioteca importada**:

  * Define las variables `CODEGEN_FCN_NAME` y `CODEGEN_DIR`, que debes actualizar según el nombre de la función generada (`myClassifierGPU`) y la ruta a tu directorio `codegen`.
  * Importa la biblioteca compartida generada (`myClassifierGPU.so`) con `add_library(... IMPORTED)` y especifica su ubicación con `set_target_properties`.
* **Incluye**: Añade al `include` path la ruta donde se encuentra `myClassifierGPU.h` dentro de la carpeta de `codegen` y las cabeceras de CUDA.
* **Ejecutable**: Crea el ejecutable `classifier` a partir de `src/classifier.cpp`.
* **Enlazado**: Enlaza contra la biblioteca generada (`myClassifierGPU`), las librerías de CUDA (`${CUDA_LIBRARIES}`), `cuDNN` y las bibliotecas de ROS.
* **Instalación**: Copia la `.so` generada a `devel/lib` y, de manera opcional, archivos auxiliares (`*.bin`) al directorio `share`.

### `package.xml`

Define el nombre (`ucsp_classify_nn`), versión, descripción, mantenedor y licencia. Declara como dependencias `roscpp`, `std_msgs`, `sensor_msgs`, `image_transport` y, en tiempo de ejecución, el nodo `usb_cam` que captura imágenes desde `/dev/video0`.

### `classifier_usb.launch`

* Lanza el nodo `classifier` del paquete `ucsp_classify_nn`.
* Permite configurar:

  * `input_topic`: tópico de entrada de la cámara.
  * `sub_h`, `sub_w`, `channels`: parámetros de recorte (ajústalos según el tamaño de entrada de tu red; por ejemplo, 227×227×3 para AlexNet, 224×224×3 para ResNet).
* Lanza el nodo `usb_cam_node` con parámetros:

  * `video_device`: dispositivo de captura (`/dev/video0`).
  * `image_width` y `image_height`: resolución de la cámara (640×360 por defecto).
  * `framerate`: 30 FPS.
  * `pixel_format`: `yuyv` (reduce latencia frente a MJPEG).
  * Desactiva `auto_white_balance` y `auto_focus` para evitar inconsistencias.

---

## ⚙️ Compilación y ejecución en Jetson

1. **Ajusta la ruta de la biblioteca** en `CMakeLists.txt`:

   * Cambia `CODEGEN_DIR` a la carpeta donde se guardó tu código generado (`myClassifierGPU/codegen/...`).
   * Verifica que `CODEGEN_FCN_NAME` coincide con el nombre de tu función (por defecto `myClassifierGPU`).

2. **Crea y construye el workspace**:

   ```bash
   cd ~/catkin_ws/src
   # Copia la carpeta ucsp_classify_nn
   catkin_make -DCMAKE_BUILD_TYPE=Release
   source ~/catkin_ws/devel/setup.bash
   ```

3. **Lanza la cámara y el clasificador**:

   ```bash
   roslaunch ucsp_classify_nn classifier_usb.launch
   ```

4. **Visualiza la clasificación** en tu PC host con MATLAB ejecutando `visualizeROSClassifier.m`, que se conectará a los tópicos `/network_view` y `/network_out` y mostrará la imagen junto con el nombre de la clase.

---

## ✅ Consejos y buenas prácticas

* **Ajustar tamaños**: Elige `sub_h` y `sub_w` según el tamaño de entrada de la red (e.g., 224 para ResNet y VGG; 227 para AlexNet).
* **Codificación de imagen**: `yuyv` reduce la latencia porque evita la descompresión JPEG; ajusta el formato en `usb_cam_node` según tu cámara.
* **Depuración**: Usa `ROS_INFO_THROTTLE` para comprobar estadísticos del buffer (mínimo, máximo y media) y validar que la imagen está correctamente escalada y recortada.
* **Gestión de memoria**: El nodo reserva y libera buffers en cada callback; si procesas a alta frecuencia y observas cuellos de botella, considera reutilizar los buffers o emplear `std::vector` con `resize` para disminuir llamadas a `new/delete`.
* **Versión de librerías**: Asegúrate de que Jetson cuenta con **CUDA**, **cuDNN** y **TensorRT** instaladas y compatibles con la versión de JetPack. Puedes verificarlo con las funciones de diagnóstico de MATLAB (`coder.checkGpuInstall`).

---

## 🧪 Flujo recomendado

1. **Guardar la red**: usa `saveNetworkToFile(net, 'miRed.mat')` para exportar tu modelo.
2. **Generar la biblioteca GPU**: ajusta `modelFile` en `myClassifierGPU_generation_script.m` y ejecútalo para obtener `myClassifierGPU.so`.
3. **Desplegar en Jetson**: copia la biblioteca y los archivos de cabecera generados y compila el nodo ROS con `catkin_make`.
4. **Probar en CPU** (opcional): ejecuta `testClassifier.m` con cualquier imagen antes de generar la biblioteca.
5. **Visualizar**: usa `visualizeROScamera.m` para comprobar la cámara y `visualizeROSClassifier.m` para ver la clasificación en tiempo real.

---

## 📑 Modelos disponibles

En la carpeta `Models/` se incluyen varios modelos preentrenados listos para usar, como **AlexNet**, **MNIST CNN**, **ResNet18/50** y **VGG16**. Puedes reemplazar o añadir tus propios modelos siguiendo el mismo flujo de generación y despliegue.

---

## 🔗 Referencias

* La estructura y el comportamiento del nodo ROS están basados en la guía de despliegue de AlexNet en Jetson Nano.
* Para detalles completos sobre requerimientos de hardware y software, revisa el README principal del proyecto.

---

