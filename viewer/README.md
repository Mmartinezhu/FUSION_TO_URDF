# URDF Viewer

Visor web local para paquetes exportados por este script.

1. En Windows, ejecuta `start_viewer.bat`.
2. Abre `http://localhost:8000` en Chrome o Edge.
3. Pulsa el boton de carpeta.
4. Selecciona la carpeta completa del paquete generado, por ejemplo `mi_robot_description`.

El visor lee `urdf/*.urdf` o `urdf/*.xacro`, resuelve mallas `package://.../meshes/*.stl` dentro de la carpeta seleccionada y muestra el robot en 3D.

Notas:
- El archivo `.urdf` puro es la ruta recomendada.
- No abras `index.html` directamente con `file://`; Chrome/Edge bloquean los modulos JavaScript locales por CORS.
- El visor usa Three.js y Lucide desde CDN, asi que necesita conexion a internet al abrirlo.
- No ejecuta ROS ni Gazebo; solo visualiza geometria, links y juntas fijas segun el URDF.
