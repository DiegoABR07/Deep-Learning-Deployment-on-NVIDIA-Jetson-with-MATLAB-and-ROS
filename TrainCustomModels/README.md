# Entrenar detectores personalizados (YOLO)

En esta carpeta encontrarás un **pipeline completo** para preparar datasets en formato COCO/Roboflow, convertirlos a tablas MATLAB, **entrenar YOLOv4** (incluida la variante *tiny*), evaluar, y luego probar el detector entrenado en imágenes o **en vivo con webcam**.

---

## 📂 Estructura

```text
TrainCustomModels/
├── coco2table.m                 # COCO/Roboflow JSON → tabla MATLAB (bbox + clases)
├── convertDataset.m             # Orquesta la conversión por splits (train/valid/test)
├── formatForYOLO.m              # Formateo de muestras para datastores combinados
├── preprocessDataYOLOv4.m       # (Opcional) Resize/augment para flujo alternativo
├── runYOLOv4.m                  # Script de experiencia: entrena y prueba
├── trainBoxesYOLOv4.m           # Función principal de entrenamiento/validación
├── webcamPredict.m              # Inferencia en vivo con webcam sobre detector entrenado
└── (salidas) checkpoints/, trainedDetectorYOLOv4.mat, curvas P-R, etc.
````

---

## 🧪 Flujo recomendado

1. **Convierte** tu dataset COCO (JSON) a tablas MATLAB: `convertDataset.m`
2. **Entrena** con `runYOLOv4.m` (internamente llama a `trainBoxesYOLOv4`).
3. **Valida/visualiza** resultados y guarda `trainedDetectorYOLOv4.mat`.
4. **Prueba** detección en imágenes o **en vivo** con `webcamPredict.m`.

---

## 📌 Scripts y funciones (entradas/salidas)

### 1) `coco2table.m` — COCO JSON → tabla MATLAB

**Qué hace:** Parsea el JSON COCO de Roboflow y construye una **tabla** con:

* `imageFilename` (`string`): ruta absoluta de cada imagen,
* `objectBoundingBoxes` (`cell{M×4 double}`): \[x y w h],
* `objectClass` (`cell{M×1 categorical}`).
  Omite imágenes sin cajas y normaliza cada bbox a fila 1×4. **Salidas seguras y consistentes** para entrenamiento.&#x20;

**Entradas**

* `jsonFile` (char/string): ruta al `_annotations.coco.json`.
* `imgFolder` (char/string): carpeta con las imágenes.

**Salida**

* `tbl` (`table`): tabla con rutas, bboxes y etiquetas.

---

### 2) `convertDataset.m` — Lanza la conversión por *splits*

**Qué hace:** Recorre `train/valid/test`, valida la existencia de `_annotations.coco.json` e imágenes, llama a `coco2table` y **guarda** en cada carpeta un `.mat` con `tbl` (por ejemplo, `train_cocoTable.mat`).&#x20;

**Entradas**

* Usa `pwd` como raíz; espera `datasets/<proyecto>/{train,valid,test}`.
* Asume nombres estándar: `_annotations.coco.json` + imágenes.

**Salidas**

* En cada split: `<split>_cocoTable.mat` con variable `tbl`.

---

### 3) `formatForYOLO.m` — *Adapter* para datastores

**Qué hace:** Adapta muestras para flujos con `combine(imageDatastore, boxLabelDatastore)`:

* Si la entrada ya viene como `{I, boxes, labels}`, la deja igual.
* Si viene como `{I, tbl}` (o `{I, {tbl}}`), **extrae** `objectBoundingBoxes` y `objectClass`.
  Valida tipos y estandariza el formato final `{I, boxes, labels}`.&#x20;

**Entradas**

* `dataIn`: `{I, boxes, labels}` **o** `{I, tbl}` / `{I, {tbl}}`.

**Salida**

* `dataOut`: `{I, boxes, labels}` listos para el *mini-batch*.

---

### 4) `preprocessDataYOLOv4.m` — (Opcional) Resize/augment

**Qué hace:** Ejemplo de *preprocess* sencillo (no usado en el flujo por defecto). Hace `imresize` a un `target` (p. ej., 416×416), ajusta cajas con el *scale* correspondiente, y devuelve `{I_resized, bboxes_rescaled}`. Útil si quieres experimentar con un pipeline manual.&#x20;

**Entradas**

* `in`: `{I, b}` donde `b` puede venir anidado en celda.
* `target`: `[H W C]` deseado (p. ej. `[416 416 3]`).

**Salida**

* `out`: `{I_resized, b_resized}`.

---

### 5) `trainBoxesYOLOv4.m` — Entrenar (¡el núcleo!)

**Qué hace (resumen de pasos):**

1. Carga las tablas `train/valid/test` y **limpia** categorías vacías.
2. **Clip/Drop** de cajas fuera del encuadre real de cada imagen.
3. Crea datastores combinados y les aplica `formatForYOLO`.
4. **Estima anchors** (9 por defecto; 6 y *split* 3+3 para *tiny-yolov4*).
5. Define el detector base (**input 416×416×3** por defecto), entrena, evalúa y genera curvas P-R.
   Además, crea/usa una carpeta `checkpoints`.&#x20;

**Entradas**

* `dataDir` (char/string): raíz con `train/valid/test` (cada split con su `*_cocoTable.mat` + imágenes).
* `checkpointsFolder` (char/string): ruta donde guardar *snapshots*.
* `modelName` (string): `"csp-darknet53-coco"` o `"tiny-yolov4-coco"`.

**Salidas**

* `detector`: objeto `yolov4ObjectDetector` listo para `detect(...)`.
* Archivos auxiliares en `checkpoints/` y métricas/figuras (P–R), según config.

**Notas clave**

* *Anchors*: ordena por área y divide en grupos (2 grupos si *tiny*, 3 si estándar).&#x20;
* *InputSize*: por defecto `[416 416 3]`; el *resize* se maneja en el *reader*.&#x20;

---

### 6) `runYOLOv4.m` — Experimento de punta a punta

**Qué hace:** Define carpetas y el `modelName`, **llama a** `trainBoxesYOLOv4` y luego **hace predicción** con el detector entrenado sobre una imagen de *test*, dibujando cajas + scores (ejemplo mínimo de *inference-after-train*).&#x20;

**Entradas**

* Variables internas: `datasetFolder`, `projectName`, `modelName`.

**Salidas**

* Variable `detector` en el *workspace*, figura con detecciones dibujadas.

---

### 7) `webcamPredict.m` — *Live demo* con webcam

**Qué hace:** Carga `trainedDetectorYOLOv4.mat` (variable `detector`), abre la webcam (Support Package), **detecta en vivo** y dibuja `label: score` en cada cuadro. Muestra FPS y permite bajar resolución para ganar rendimiento.&#x20;

**Entradas**

* `trainedDetectorYOLOv4.mat` en la misma carpeta (variable `detector`).

**Salidas**

* Ventana con *preview* y detecciones en tiempo real (FPS en título).

---

## 🧩 Piezas relacionadas (detección e inferencia)

Aunque el foco aquí es **entrenar**, también se incluyen utilidades de inferencia genéricas (útiles para validar tus modelos):

* `myDetector.m`: detector **CPU**; garantiza RGB, llama a `detect(det, I, Threshold=...)`. **Salidas**: `[bboxes, scores, labels]` (labels como *cellstr*).&#x20;
* `myDetectorGPU.m`: detector **GPU/Codegen** con firma `[b,s,lIdx] = myDetectorGPU(im, detectorFile, threshold)`. Carga el detector una vez, asegura 3 canales y devuelve **índices** de clase (1-based).&#x20;
* `myDetectorGPU_generation_script.m`: prepara **codegen** para Jetson (dll + *Embedded Coder*), fija `inputSize` desde el detector, define tipos y genera `myDetectorGPU.so`.&#x20;
* `testDetector.m`: *harness* para comparar CPU/GPU y **normalizar etiquetas** a texto antes de dibujar anotaciones.&#x20;

> Si vas a desplegar en ROS (fuera de esta carpeta), tus nodos C++ publicarán:
>
> * `/network_view` (imagen RGB recortada) y
> * `/network_detections` (Float32MultiArray con filas `[x y w h score labelIdx]`), que puedes **visualizar desde MATLAB** con `visualizeROSDetector.m`.&#x20;

---

## 🛠️ Consejos prácticos

* **Calidad de anotaciones**: ejecuta *clip/drop* (ya incluido) para evitar cajas fuera de imagen y *anchors* más estables.&#x20;
* **Anchors y tamaños**: usa 416×416×3 como base (o ajusta a tu *backbone*). Estima anchors tras el *resize* esperado.&#x20;
* **`formatForYOLO`** te quita dolores de cabeza con las combinaciones `{I, tbl}` → `{I, boxes, labels}` en *mini-batches*.&#x20;
* **Webcam**: baja resolución si tu cámara es 1080p/4K; ganarás FPS.&#x20;

---

## ▶️ Ejemplos rápidos

### Entrenar (fin a fin) y probar una imagen

```matlab
% 1) Asegúrate de tener datasets/<proyecto>/{train,valid,test}
convertDataset        % crea train_cocoTable.mat, etc.

% 2) Lanza experimento
runYOLOv4             % entrena y prueba sobre una imagen de test
```

&#x20;

### Cargar detector y detectar en vivo (webcam)

```matlab
% tras el entrenamiento, asegúrate de tener trainedDetectorYOLOv4.mat
webcamPredict
```



---

## 📦 Salidas típicas

* `trainedDetectorYOLOv4.mat` (variable `detector` lista para `detect(...)`).&#x20;
* Carpeta `checkpoints/<proyecto>/` con snapshots y logs de entrenamiento.&#x20;
* Figuras/curvas de evaluación (P–R, *mean IoU* de anchors, etc.).&#x20;

```

¿Quieres que deje al final una **sección breve de “Problemas comunes y soluciones”** (por ejemplo: “no encuentra el JSON”, “anchors raros”, “out of memory”)? La agrego en un periquete.
```


Aquí tienes el **README para la carpeta TrainCustomModels**, listo para copiar y pegar. Explica de forma clara qué hace cada archivo, sus entradas y salidas, y cómo usarlos juntos para entrenar y probar un detector YOLOv4 personalizado en MATLAB.

````markdown
# Entrenar detectores personalizados con YOLOv4

Esta carpeta proporciona un **pipeline completo** para crear tu propio detector de objetos con YOLOv4 (incluyendo la variante *tiny*) a partir de datos anotados en formato COCO/Roboflow. Los scripts convierten JSON a tablas MATLAB, preparan los datos, entrenan el modelo, lo evalúan y permiten probarlo en imágenes individuales o en vivo mediante webcam.

---

## 📂 Estructura de la carpeta

```text
TrainCustomModels/
├── coco2table.m                # COCO/Roboflow JSON → tabla MATLAB (bbox + clases)
├── convertDataset.m            # Orquesta la conversión por splits (train/valid/test)
├── formatForYOLO.m             # Formateo de muestras para datastores combinados
├── preprocessDataYOLOv4.m      # (Opcional) Resize/augment para flujo alternativo
├── runYOLOv4.m                 # Script de experiencia: entrena y prueba
├── trainBoxesYOLOv4.m          # Función principal de entrenamiento y evaluación
├── webcamPredict.m             # Inferencia en vivo con webcam sobre detector entrenado
└── (salidas)                   # Checkpoints, trainedDetectorYOLOv4.mat, curvas P-R, etc.
````

---

## 🛠️ Preparación del dataset

### `coco2table.m` — COCO JSON → tabla MATLAB

Convierte un archivo JSON de anotaciones COCO/Roboflow en una **tabla MATLAB** con tres columnas: `imageFilename` (ruta absoluta de la imagen), `objectBoundingBoxes` (celda de M×4 `[x y w h]` para cada imagen) y `objectClass` (celda de M etiquetas como `categorical`). Omite imágenes sin cajas y garantiza que cada bbox sea una fila 1×4.
**Entradas:** `jsonFile` (ruta al `_annotations.coco.json`), `imgFolder` (carpeta de imágenes).
**Salida:** `tbl` (tabla con rutas, bboxes y etiquetas).

### `convertDataset.m` — Conversión por splits

Automatiza la conversión de todo tu dataset. Recorre las carpetas `train`, `valid` y `test`, verifica la existencia del JSON y las imágenes, llama a `coco2table` y **guarda** en cada carpeta un archivo `<split>_cocoTable.mat` con la tabla correspondiente.
**Entradas:** se define `dataDir` con estructura `datasets/<proyecto>/{train,valid,test}`.
**Salidas:** `train_cocoTable.mat`, `valid_cocoTable.mat`, `test_cocoTable.mat`.

---

## 🧰 Formateo para YOLO

### `formatForYOLO.m` — Adaptador para datastores

Convierte lotes `{I, boxes, labels}` o `{I, tbl}` a `{I, boxes, labels}`. Si la entrada es `{I, tbl}` (o `{I, {tbl}}`), extrae `objectBoundingBoxes` y `objectClass` de la tabla; si ya viene en forma `{I, boxes, labels}`, no hace cambios.
**Entrada:** celda `dataIn` con 2 o 3 elementos.
**Salida:** celda `{I, boxes, labels}` lista para `combine`/`transform`.

### `preprocessDataYOLOv4.m` — Resize/augment (opcional)

Ejemplo de preprocesado: hace `imresize` a un tamaño objetivo (`target`), convierte la imagen a `single`, reescala las cajas de acuerdo con el factor de resize y devuelve `{I_resized, bboxes_rescaled}`.
**Entradas:** `in = {I, b}` (imagen y cajas), `target = [H W C]`.
**Salida:** `out = {I_resized, b_resized}`.

---

## 🎓 Entrenamiento y evaluación

### `trainBoxesYOLOv4.m` — Núcleo del entrenamiento

Función que entrena y evalúa un detector YOLOv4 (MATLAB R2023a o posterior). Pasos principales:

1. **Carga y limpieza de tablas**: lee `train/valid/test_cocoTable.mat` de `dataDir` y elimina categorías vacías.
2. **Recorte/Descarte de cajas**: recorta las cajas al tamaño de la imagen real o descarta cajas fuera del encuadre; muestra cuántas cajas se eliminaron por split.
3. **Creación de datastores**: combina `imageDatastore` y `boxLabelDatastore`, aplicando `formatForYOLO` para obtener `{I, boxes, labels}`.
4. **Estimación de anchors**: redimensiona las cajas a 416×416 y estima anchors (9 por defecto). Para el modelo *tiny*, calcula 6 anchors y los divide en dos grupos de 3.
5. **Definición del detector base**: usa `yolov4ObjectDetector(modelName, classNames, anchorBoxes, InputSize=[416 416 3])` para crear el modelo.
6. **Opciones de entrenamiento**: configura `trainingOptions("adam", ...)` con parámetros de mini-batch, tasa de aprendizaje inicial, número de épocas (120 por defecto), regularización L2, uso de GPU, validación, checkpoints y gráfica de progreso.
7. **Entrenamiento**: llama a `trainYOLOv4ObjectDetector` para entrenar y guarda `trainedDetectorYOLOv4.mat`.
8. **Evaluación**: detecta objetos en `dsTest`, calcula *mAP\@0.5*, `recall` y `precision` y lo muestra en pantalla. También genera curvas Precisión–Recall para cada clase.

**Entradas:**

* `dataDir`: carpeta base con `train`, `valid`, `test` (cada split con su .mat y sus imágenes).
* `checkpointsFolder`: carpeta donde se guardan los modelos intermedios.
* `modelName`: cadena (`"csp-darknet53-coco"` para el modelo completo, `"tiny-yolov4-coco"` para la variante ligera).

**Salidas:**

* `detector`: objeto `yolov4ObjectDetector` listo para inferencia.
* Archivos en `checkpointsFolder`, métricas e imágenes generadas.

---

### `runYOLOv4.m` — Experimento de punta a punta

Script que sirve de plantilla rápida para entrenar y probar el detector:

1. **Define** rutas (`datasetFolder`, `projectName`), elige el modelo (`modelName` = `"tiny-yolov4-coco"` por defecto) y crea la carpeta de checkpoints.
2. **Entrena**: llama a `trainBoxesYOLOv4(dataDir, checkpointsFolder, modelName)`.
3. **Prueba**: lee una imagen de la carpeta `test`, ejecuta `detect(detector,I,Threshold=0.20)` y dibuja las cajas con sus puntuaciones.

Es ideal para replicar rápidamente el proceso con tus propios datos; basta cambiar `projectName` y `modelName`.

---

### `webcamPredict.m` — Demo en vivo

Permite probar el detector entrenado en tiempo real usando una webcam. Pasos principales:

1. **Carga** el archivo `trainedDetectorYOLOv4.mat` (variable `detector`).
2. **Conecta** a la webcam y ajusta su resolución a la más baja disponible para aumentar FPS.
3. **Bucle principal**: captura una imagen (`snapshot(cam)`), ejecuta `detect(detector,I,Threshold=0.70)`, dibuja las cajas con `insertObjectAnnotation` y muestra FPS actualizado.
4. **Salida**: cierra al pulsar Q y libera la webcam.

Perfecto para demostrar rápidamente el rendimiento del detector y ajustar el umbral de confianza.

---

## 📦 Salidas típicas

Al ejecutar el pipeline se generan:

* `trainedDetectorYOLOv4.mat`: archivo MATLAB con la variable `detector` lista para `detect(...)`.
* Carpeta `checkpoints/<proyecto>/`: snapshots del entrenamiento y logs, útiles para reanudar o analizar.
* Figuras de evaluación (curvas Precisión–Recall, IoU de anchors, etc.).
* `*_cocoTable.mat` en cada split de tu dataset (train/valid/test) con las tablas generadas.

---

## 💡 Consejos prácticos

* **Anota bien**: revisa que tus archivos JSON COCO y las imágenes estén correctamente alineados antes de convertirlos.
* **Anchors y resolución**: usa 416×416×3 como tamaño de entrada por defecto; ajusta anchors si cambias la resolución base.
* **Calidad de las cajas**: el script recorta o descarta cajas fuera del encuadre y evita cajas de 1 px para robustez.
* **Webcam**: reduce la resolución de la webcam para mejorar la velocidad de inferencia.
* **Tiny vs. full**: la variante *tiny-yolov4-coco* usa 6 anchors (dos grupos de 3) y es más ligera; el modelo completo usa 9 anchors (tres grupos).