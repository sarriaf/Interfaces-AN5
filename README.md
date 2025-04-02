# Interfaces-AN5

Este repositorio contiene las interfaces necesarias para la comunicación y control del robot AN5 en ROS2 utilizando AppDesigner y Unity. A continuación, se detallan los pasos para la instalación, configuración y modificación de los archivos requeridos.


## ✅ Requisitos del sistema

Antes de comenzar, asegúrate de contar con el siguiente entorno de desarrollo instalado:

- 🐧 **Ubuntu 22.04**
- 🤖 **ROS 2 Humble Hawksbill**
- 🧠 **MATLAB R2024b** o superior
- 🎮 **Unity 2022.3.42f1** o superior

---
---
---
---
---

## 📥 Clonación del Repositorio

Para comenzar, clona este repositorio en tu máquina local ejecutando el siguiente comando:

```bash
cd ~
git clone https://github.com/sarriaf/Interfaces-AN5.git
```

## 🏗 Creación del Workspace de ROS2

1. Dentro de tu **home**, crea un workspace de ROS2:
   
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Copia los paquetes de ROS2 de la API llamados **fr_ros2** y **frhal_msgs** del repositorio clonado hacia el workspace de ROS2:
   
   ```bash
   cp -r ~/Interfaces-AN5/fr_ros2 ~/ros2_ws/src/
   cp -r ~/Interfaces-AN5/frhal_msgs ~/ros2_ws/src/
   ```

3. Copia el paquete **code** dentro del workspace:
   
   ```bash
   cp -r ~/Interfaces-AN5/code ~/ros2_ws/src/
   ```

4. Compila los paquetes uno por uno:
   
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select frhal_msgs
   ```

   Después de esperar que finalice la compilación compila el siguiente paquete:

   ```bash
   colcon build --packages-select fr_ros2
   ```

   Y al finalizar esta compilación, ejecuta el siguiente comando para compilar el paquete **code** y para verificar que las anteriores compilaciones sean correctas utilizando el siguiente comando:
   ```bash
   colcon build
   ```

## 🔧 Modificación de Archivos

### 1️⃣ Modificación del Archivo **ROS_API**

Para realizar esta modificación, ejecuta el siguiente comando en la terminal:

```bash
gedit ~/ros2_ws/src/fr_ros2/src/ROS_API.cpp
```

Una vez abierto el archivo en el editor de texto, localiza las líneas **46**, **47**, **59** y **78**, y reemplaza el valor `0` por `2`, tal como se muestra a continuación:

### Antes:

```cpp
46    this->declare_parameter<uint8_t>("toolcoord_install",0);//默认工具安装在机器人末端
47    this->declare_parameter<uint8_t>("toolcoord_type",0);//默认是工具坐标系
59    this->declare_parameter<int>("MoveJLC_tool",0);
78    this->declare_parameter<int>("Spline_tool",1);
```

### Después:

```cpp
46    this->declare_parameter<uint8_t>("toolcoord_install",2);//默认工具安装在机器人末端
47    this->declare_parameter<uint8_t>("toolcoord_type",2);//默认是工具坐标系
59    this->declare_parameter<int>("MoveJLC_tool",2);
78    this->declare_parameter<int>("Spline_tool",2);
```

Guarda los cambios realizados y compila el paquete con los siguientes comandos:

```bash
cd ~/ros2_ws
colcon build
```

### 2️⃣ Modificación del Archivo **inverse_kinematics** en AppDesigner

Ubica el archivo dentro de la carpeta **Interfaz AppDesigner AN5** y modifica las siguientes líneas para ajustar la ruta del script Python:

```matlab
% (C) Ejecutar el script Python
scriptPath = "/home/usuario/ros2_ws/src/code/code/MoveL.py";
cmd = "bash -c 'export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6; " + ...
      "source /opt/ros/humble/setup.bash; " + ...
      "source /home/usuario/ros2_ws/install/setup.bash; " + ...
      "python3 " + scriptPath + "'";
[status, cmdout] = system(cmd);
```

🔹 **Nota:** Reemplaza `/home/usuario/ros2_ws/` con la ruta real de tu workspace de ROS2, esta modificación debe hacerse en **dos partes diferentes** dentro del código.

### 3️⃣ Modificación de scripts en la interfaz de Unity

Ubica y edita los siguientes scripts dentro de la carpeta **Interfaz Unity AN5**:

- `Control Articular`
- `CartesianStateWritterNew`
- `Record_Panel`
- `SendTxt`

Cada uno de estos scripts contiene rutas de archivos que deben actualizarse para apuntar correctamente a la ubicación de la carpeta **Interfaz AppDesigner AN5** en tu sistema.

Debes modificar la línea correspondiente a la ruta del archivo en cada script:

- `Control Articular` → línea **70**
- `CartesianStateWritterNew` → línea **67**
- `Record_Panel` → línea **48**
- `SendTxt` → línea **29**

🔹 **Nota:** Asegúrate de reemplazar la ruta existente con la ubicación real de la carpeta **Interfaz AppDesigner AN5** en tu equipo, manteniendo el formato correcto de la cadena de texto.

💾 **Guarda todos los scripts editados** para que Unity reconozca los cambios correctamente.

## 🌐 Instalación de rosbridge_suite

`rosbridge_suite` permite la comunicación entre ROS 2 y aplicaciones externas (como Unity o interfaces web) a través de WebSocket. Es esencial para integrar la interfaz Unity AN5 con ROS 2.

```bash
sudo apt install ros-humble-rosbridge-server
```

---
---
---
---

### 🎥 Video tutorial del proceso de configuración

Para una guía visual completa, puedes consultar el siguiente video donde se muestra el paso a paso de todas las configuraciones mencionadas en este documento:

🔗 [Ver video del paso a paso](#) <!-- Reemplaza '#' con tu enlace -->

---
---
---
---

### ▶️ Ejecución del sistema

Una vez realizada toda la configuración, puedes seguir las instrucciones del siguiente video para ejecutar correctamente el sistema completo (Unity + AppDesigner + ROS 2):

🔗 [Ver video de ejecución](https://www.youtube.com/watch?v=qW5HY1y-aKo) <!-- Reemplaza '#' con tu enlace -->

---
