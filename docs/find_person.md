# FindPerson: funcionamiento actual

Este documento describe el flujo actual de `find_person` y los cambios hechos
para depurar y estabilizar la deteccion, registro facial y guardado en
knowledge.

## Archivos principales

- `src/Executor/executor_tools/src/FindPerson.cpp`: controla la action
  `find_person`, mueve la cabeza, llama a vision y actualiza knowledge.
- `src/Vision/vision_tools/src/Vision.cpp`: wrapper C++ para los servicios de
  vision usados por el executor.
- `src/Vision/face_recognizer/face_recog/face_recog/face_recogFlorence2.py`:
  servidor de `detect_person`, `identify_person`, `describe_person` y
  `register_person`.

## Flujo general

1. `FindPerson` recibe el argumento de la action, por ejemplo `anybody_1` o
   `guest_1`.
2. Construye una lista de posiciones de escaneo para la cabeza.
3. En cada posicion:
   - mueve cabeza con `MoveHead`;
   - espera estabilizacion;
   - llama a `DetectPersonPose`;
   - si detecta una cara, convierte bbox a pose con `BboxToPose`;
   - apunta a la persona con `LookAt`;
   - si la cara quedo baja en la imagen, baja un poco mas la cabeza;
   - refresca el frame de rostro antes de registrar;
   - intenta identificar;
   - si no identifica y es `anybody`, registra como `guest_N`;
   - describe a la persona;
   - actualiza `knowledge_manager`.
4. Si no encuentra persona en ninguna posicion, regresa la cabeza a
   `pan=0.0`, `tilt=0.0` y termina con fallo.

## Barridos actuales

Para `anybody` o `anybody_N`, el escaneo es vertical al frente:

```text
pan=0.0 tilt=0 deg
pan=0.0 tilt=-10 deg
pan=0.0 tilt=-20 deg
pan=0.0 tilt=-30 deg
```

Para persona especifica, el escaneo hace dos barridos laterales:

```text
Barrido 1, cabeza recta:
pan=0.0  tilt=0 deg
pan=-0.6 tilt=0 deg
pan=0.6  tilt=0 deg

Barrido 2, cabeza baja:
pan=0.0  tilt=-25 deg
pan=-0.6 tilt=-25 deg
pan=0.6  tilt=-25 deg
```

El centro no se repite al final del barrido. Si no hay deteccion, la cabeza
vuelve a reposo al terminar la action.

## Timeouts e intentos

Cada posicion de escaneo usa un timeout explicito de `3 s` para
`DetectPersonPose`.

Dentro de `DetectPersonPose`:

- `DetectPerson` se reintenta hasta que se agote el timeout de la posicion.
- Cada llamada individual a `DetectPerson` usa timeout interno de `1 s`.
- Entre intentos fallidos hay una pausa de `200 ms`.
- La cabeza no se mueve dentro de esos intentos; solo cambia el frame de la
  camara.

Si se detecta cara pero `BboxToPose` falla por nube/TF, tambien se reintenta
dentro de la misma ventana de tiempo.

Antes de `IdentifyPerson` y `RegisterPerson`, `FindPerson` hace un refresh de
la cara con hasta `5` intentos de `DetectPerson`. Este refresh actualiza el
frame que `register_person` usara para guardar la imagen facial.

## Registro facial

`face_recogFlorence2.py` mantiene un cache corto del ultimo `detect_person`
exitoso:

- frame completo;
- bbox de la cara elegida;
- numero de frame;
- tiempo de deteccion.

El cache dura `5 s`. `register_person` usa ese cache primero para evitar que el
registro dependa de una segunda deteccion varios frames despues. Si el cache
expira o no existe, `register_person` conserva el comportamiento anterior:
detecta en el frame actual.

El archivo facial se guarda en:

```bash
~/vision/faces_detected/guest_1.png
```

Para otros invitados cambia el sufijo, por ejemplo `guest_2.png`.

## Knowledge

El guardado en knowledge es independiente del registro facial.

`FindPerson` actualiza:

- `name`;
- `description`;
- `navigation`.

El runtime se puede revisar aqui:

```bash
install/knowledge_bank/share/knowledge_bank/runtime/runtime.yaml
```

Comando util:

```bash
grep -n -A10 "name: guest_1" install/knowledge_bank/share/knowledge_bank/runtime/runtime.yaml
```

## Imagen debug

El topico de imagen debug para `find_person` es:

```bash
/face_recog/debug_image
```

Se publica cuando hay deteccion, identificacion o registro facial. Se puede ver
con:

```bash
ros2 run rqt_image_view rqt_image_view /face_recog/debug_image
```

## Logs esperados

Se dejaron en `INFO` los mensajes importantes:

```text
[FindPerson] scanPosition ...
[FindPerson] DetectPersonPose search timeout=3 s
DetectPersonPose: no face detected within 3 s
[FindPerson] DetectPersonPose OK ...
[FindPerson] Face is low in image ... lowering head for capture
[FindPerson] Refreshed face frame before identify/register ...
RegisterPerson OK ...
[FindPerson] Knowledge updated ...
[find_person] SUCCESS/FAILED
```

Los mensajes repetitivos por intento fallido quedaron en `DEBUG` para no
saturar la terminal.

## Prueba por action

```bash
source install/setup.bash
ros2 action send_goal /executor/execute_task executor_interfaces/action/ExecuteTask "{task: {step_id: 1, agent: 'Justina', action: 'find_person', args: ['anybody_1']}}" --feedback
```

Despues de probar, verificar:

```bash
ls -lh ~/vision/faces_detected/guest_1.png
grep -n -A10 "name: guest_1" install/knowledge_bank/share/knowledge_bank/runtime/runtime.yaml
```
