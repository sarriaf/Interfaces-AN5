/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using System.Collections;

public class ControlArticular : MonoBehaviour
{
    // Referencias a los sliders de las articulaciones
    public Slider[] jointSliders;
    
    // Campos de entrada para mostrar e ingresar valores de los sliders
    public InputField[] jointValueInputs;
    
    // Botones para aumentar y disminuir el valor de cada articulación
    public Button[] increaseButtons;
    public Button[] decreaseButtons;

    // Slider y campo de entrada para controlar la velocidad
    public Slider speedSlider;
    public InputField speedValueInput;
    
    // Botones para ajustar la velocidad
    public Button increaseSpeedButton;
    public Button decreaseSpeedButton;

    // Campo de entrada para el tiempo de espera en segundos
    public InputField delayInputField;
    
    // Botones para enviar, añadir, eliminar y guardar puntos
    public Button sendButton;
    public Button addButton;
    public Button removeButton;
    public Button saveTxtButton;
    
    // Texto para mostrar los puntos añadidos
    public Text coordinatesDisplay;
    
    // Referencias a componentes de ROS2 y suscriptores
    public Ros2CommandSender ros2CommandSender;
    public JointPositionSubscriber jointPositionSubscriber;
    public JointStateWriter[] jointStateWriters;
    public MGD_Subscriber MGD_Subscriber; // Suscriptor para resultados de cinemática directa

    // Variables para almacenar estados y configuraciones
    private float[] jointPositions = new float[6];
    private float speed = 10f; // Velocidad por defecto
    private float delay = 0f; // Tiempo de espera por defecto
    private bool isArticularModeActive = false;
    
    // Listas para almacenar posiciones articulares, velocidades y delays
    private List<float[]> jointPositionsList = new List<float[]>();
    private List<float> speedList = new List<float>();
    private List<float> delayList = new List<float>();
    
    // Diccionario para almacenar resultados de cinemática directa asociados a puntos
    private Dictionary<int, string> pointToDirectaResult = new Dictionary<int, string>();
    
    // Cola para índices pendientes de recibir resultados de MGD
    private Queue<int> pendingMGDIndices = new Queue<int>();

    // Ruta donde se guardará el archivo de posiciones
    private string savePath = "/home/tarw/Interfaz AppDesigner AN5";

    void Start()
    {
        // Asignar listeners a los botones principales
        if (sendButton != null)
        {
            sendButton.onClick.AddListener(() =>
            {
                StartCoroutine(SendSplineCommandsWithDelay());
            });
        }

        if (addButton != null)
        {
            addButton.onClick.AddListener(AddJointPosition);
        }

        if (removeButton != null)
        {
            removeButton.onClick.AddListener(RemoveLastJointPosition);
        }

        if (saveTxtButton != null)
        {
            saveTxtButton.onClick.AddListener(GuardarEnTxt);
        }

        SetSliderLimits(); // Establecer límites para los sliders de articulaciones

        // Configurar eventos para sliders y campos de entrada de articulaciones
        for (int i = 0; i < jointSliders.Length; i++)
        {
            int index = i;
            jointSliders[i].onValueChanged.AddListener(value => OnSliderValueChanged(index, value));
            jointValueInputs[i].onEndEdit.AddListener(value => OnInputFieldValueChanged(index, value));
        }

        // Configurar eventos para botones de aumentar y disminuir articulaciones
        for (int i = 0; i < increaseButtons.Length; i++)
        {
            int index = i;
            increaseButtons[i].onClick.AddListener(() => IncreaseJointValue(index));
            decreaseButtons[i].onClick.AddListener(() => DecreaseJointValue(index));
        }

        // Configurar eventos para botones de velocidad
        if (increaseSpeedButton != null)
        {
            increaseSpeedButton.onClick.AddListener(IncreaseSpeedValue);
        }

        if (decreaseSpeedButton != null)
        {
            decreaseSpeedButton.onClick.AddListener(DecreaseSpeedValue);
        }

        // Configurar slider y campo de entrada de velocidad
        if (speedSlider != null)
        {
            speedSlider.minValue = 1f;
            speedSlider.maxValue = 100f;
            speedSlider.wholeNumbers = true;
            speedSlider.value = speed;
            speedSlider.onValueChanged.AddListener(OnSpeedSliderValueChanged);
        }

        if (speedValueInput != null)
        {
            speedValueInput.text = speed.ToString();
            speedValueInput.onEndEdit.AddListener(OnSpeedInputFieldValueChanged);
        }

        // Configurar campo de entrada de delay
        if (delayInputField != null)
        {
            delayInputField.text = delay.ToString();
            delayInputField.onEndEdit.AddListener(OnDelayInputFieldValueChanged);
            delayInputField.interactable = false; // Deshabilitar por defecto
        }

        // Suscribirse a eventos de actualización de posiciones articulares
        jointPositionSubscriber.OnJointPositionsUpdated += UpdateInitialJointPositions;

        // Suscribirse a eventos de resultados de cinemática directa
        if (MGD_Subscriber != null)
        {
            MGD_Subscriber.OnInverseKinematicsResultReceived += ReceiveDirectaKinematicsResult;
            Debug.Log("ControlArticular suscrito a OnInverseKinematicsResultReceived.");
        }
        else
        {
            Debug.LogError("MGD_Subscriber no está asignado en el inspector.");
        }

        // Iniciar actualizaciones de posiciones articulares
        jointPositionSubscriber.StartUpdating();

        // FORZAR actualización inicial de los inputs (sliders y campos)
        UpdateInitialJointPositions(jointPositionSubscriber.GetLastKnownPositions());
    }

    // Establecer límites para los sliders de articulaciones
    private void SetSliderLimits()
    {
        jointSliders[0].minValue = -90f;
        jointSliders[0].maxValue = 90f;
        jointSliders[1].minValue = -145f;
        jointSliders[1].maxValue = -45f;
        jointSliders[2].minValue = -152f;
        jointSliders[2].maxValue = 152f;
        jointSliders[3].minValue = -233f;
        jointSliders[3].maxValue = 89f;
        jointSliders[4].minValue = -160f;
        jointSliders[4].maxValue = 160f;
        jointSliders[5].minValue = -179f;
        jointSliders[5].maxValue = 90f;
    }

    // Manejar el cambio de valor en los sliders de articulaciones
    private void OnSliderValueChanged(int index, float value)
    {
        if (!isArticularModeActive)
        {
            isArticularModeActive = true;
            jointPositionSubscriber.StopUpdating(); // Detener actualizaciones automáticas
        }

        float roundedValue = Mathf.Round(value * 100f) / 100f; // Redondear a dos decimales
        jointPositions[index] = roundedValue;
        jointSliders[index].value = roundedValue; // Actualizar slider con valor redondeado
        jointStateWriters[index].Write(roundedValue * Mathf.Deg2Rad); // Actualizar URDF

        if (jointValueInputs != null && jointValueInputs.Length > index)
        {
            jointValueInputs[index].text = $"{roundedValue}";
        }

        Debug.Log($"Articulación {index + 1} ajustada a {jointPositions[index]}°.");
    }

    // Manejar el cambio de valor en los campos de entrada de articulaciones
    private void OnInputFieldValueChanged(int index, string value)
    {
        if (float.TryParse(value, out float result))
        {
            result = Mathf.Clamp(result, jointSliders[index].minValue, jointSliders[index].maxValue);
            jointPositions[index] = result;
            jointSliders[index].value = result; // Actualizar slider con valor ingresado
            jointStateWriters[index].Write(result * Mathf.Deg2Rad); // Actualizar URDF
            Debug.Log($"Articulación {index + 1} ajustada a {result}° desde el campo de entrada.");
        }
    }

    // Aumentar el valor de una articulación específica
    private void IncreaseJointValue(int index)
    {
        float newValue = jointPositions[index] + 1f;
        newValue = Mathf.Clamp(newValue, jointSliders[index].minValue, jointSliders[index].maxValue);
        jointPositions[index] = newValue;
        jointSliders[index].value = newValue;
        jointStateWriters[index].Write(newValue * Mathf.Deg2Rad);

        if (jointValueInputs != null && jointValueInputs.Length > index)
        {
            jointValueInputs[index].text = $"{newValue}";
        }

        Debug.Log($"Articulación {index + 1} incrementada a {newValue}°.");
    }

    // Disminuir el valor de una articulación específica
    private void DecreaseJointValue(int index)
    {
        float newValue = jointPositions[index] - 1f;
        newValue = Mathf.Clamp(newValue, jointSliders[index].minValue, jointSliders[index].maxValue);
        jointPositions[index] = newValue;
        jointSliders[index].value = newValue;
        jointStateWriters[index].Write(newValue * Mathf.Deg2Rad);

        if (jointValueInputs != null && jointValueInputs.Length > index)
        {
            jointValueInputs[index].text = $"{newValue}";
        }

        Debug.Log($"Articulación {index + 1} decrementada a {newValue}°.");
    }

    // Manejar el cambio de valor en el slider de velocidad
    private void OnSpeedSliderValueChanged(float value)
    {
        speed = Mathf.Round(value); // Redondear al entero más cercano
        speedSlider.value = speed; // Actualizar slider con valor redondeado
        if (speedValueInput != null)
        {
            speedValueInput.text = speed.ToString();
            Debug.Log($"Velocidad ajustada a: {speed}");
        }
    }

    // Manejar el cambio de valor en el campo de entrada de velocidad
    private void OnSpeedInputFieldValueChanged(string value)
    {
        if (float.TryParse(value, out float result))
        {
            result = Mathf.Round(result); // Redondear al entero más cercano
            result = Mathf.Clamp(result, speedSlider.minValue, speedSlider.maxValue);
            speed = result;
            speedSlider.value = result; // Actualizar slider con nuevo valor
            Debug.Log($"Velocidad configurada a {speed}.");
        }
        else
        {
            speedValueInput.text = speed.ToString(); // Revertir a último valor válido
        }
    }

    // Aumentar la velocidad en 1 unidad
    private void IncreaseSpeedValue()
    {
        float newSpeed = speed + 1f;
        newSpeed = Mathf.Clamp(newSpeed, speedSlider.minValue, speedSlider.maxValue);
        speed = newSpeed;
        speedSlider.value = newSpeed;
        speedValueInput.text = newSpeed.ToString();
        Debug.Log($"Velocidad incrementada a {newSpeed}.");
    }

    // Disminuir la velocidad en 1 unidad
    private void DecreaseSpeedValue()
    {
        float newSpeed = speed - 1f;
        newSpeed = Mathf.Clamp(newSpeed, speedSlider.minValue, speedSlider.maxValue);
        speed = newSpeed;
        speedSlider.value = newSpeed;
        speedValueInput.text = newSpeed.ToString();
        Debug.Log($"Velocidad decrementada a {newSpeed}.");
    }

    // Manejar el cambio de valor en el campo de entrada de delay
    private void OnDelayInputFieldValueChanged(string value)
    {
        if (float.TryParse(value, out float result))
        {
            delay = Mathf.Max(0, result); // Asegurar que el delay no sea negativo
            Debug.Log($"Tiempo de espera configurado a {delay} segundos.");
        }
        else
        {
            delayInputField.text = delay.ToString(); // Revertir a último valor válido
        }
    }

    // Actualizar las posiciones iniciales de las articulaciones desde el suscriptor
    private void UpdateInitialJointPositions(float[] initialPositions)
    {
        for (int i = 0; i < initialPositions.Length; i++)
        {
            float roundedValue = Mathf.Round(initialPositions[i] * 100f) / 100f; // Redondear a dos decimales

            jointSliders[i].value = roundedValue;  // Actualizar slider con valor recibido
            jointPositions[i] = roundedValue;      // Guardar en el array de posiciones

            jointStateWriters[i].Write(roundedValue * Mathf.Deg2Rad);  // Actualizar URDF

            if (jointValueInputs != null && jointValueInputs.Length > i)
            {
                jointValueInputs[i].text = roundedValue.ToString(); // Actualizar campo de entrada
            }
        }
    }

    // Añadir una posición articular a la lista y enviarla a ROS2
    private void AddJointPosition()
    {
        jointPositionsList.Add((float[])jointPositions.Clone());
        speedList.Add(speed);

        // Guardar el delay si está activo, de lo contrario 0
        float pointDelay = (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive()) ? DelayModeController.Instance.GetDelayTime() : 0f;
        delayList.Add(pointDelay);

        // Enviar las posiciones articulares al tópico de cinemática directa
        string jointPositionsString = string.Join(",", jointPositions);
        ros2CommandSender.SendCommandToTopic("/input_joint_position", jointPositionsString);
        Debug.Log($"Enviado al tópico de cinemática directa (/input_joint_position): {jointPositionsString}");

        // Agregar el índice a la cola de pendientes
        pendingMGDIndices.Enqueue(jointPositionsList.Count - 1);

        // Actualizar la visualización de puntos
        UpdateCoordinatesDisplay();
    }

    // Eliminar la última posición articular de la lista
    private void RemoveLastJointPosition()
    {
        if (jointPositionsList.Count > 0)
        {
            jointPositionsList.RemoveAt(jointPositionsList.Count - 1);
            speedList.RemoveAt(speedList.Count - 1);
            delayList.RemoveAt(delayList.Count - 1);

            int removedIndex = jointPositionsList.Count;
            if (pointToDirectaResult.ContainsKey(removedIndex))
            {
                pointToDirectaResult.Remove(removedIndex);
            }

            // Remover el índice de la cola si está pendiente
            if (pendingMGDIndices.Contains(removedIndex))
            {
                Queue<int> tempQueue = new Queue<int>();
                while (pendingMGDIndices.Count > 0)
                {
                    int index = pendingMGDIndices.Dequeue();
                    if (index != removedIndex)
                    {
                        tempQueue.Enqueue(index);
                    }
                }
                pendingMGDIndices = tempQueue;
            }

            // Actualizar la visualización de puntos
            UpdateCoordinatesDisplay();

            Debug.Log($"Se eliminó la última posición guardada (Punto {removedIndex}).");
        }
        else
        {
            Debug.LogWarning("No hay posiciones guardadas para eliminar.");
        }
    }

    // Recibir y almacenar el resultado de cinemática directa
    private void ReceiveDirectaKinematicsResult(string messageData)
    {
        Debug.Log("ControlArticular recibió resultado de MGD: " + messageData);

        if (string.IsNullOrEmpty(messageData))
        {
            Debug.LogWarning("Mensaje de cinemática directa es nulo o vacío.");
            return;
        }

        if (pendingMGDIndices.Count > 0)
        {
            int index = pendingMGDIndices.Dequeue();
            if (!pointToDirectaResult.ContainsKey(index))
            {
                pointToDirectaResult.Add(index, messageData);
                Debug.Log($"Resultado de cinemática directa recibido para punto {index} y almacenado.");
            }
            else
            {
                Debug.LogWarning($"El punto {index} ya tiene un resultado de cinemática directa asociado.");
            }

            // Actualizar la visualización con el resultado de MGD
            UpdateCoordinatesDisplay();
        }
        else
        {
            Debug.LogWarning("No hay índices pendientes para asignar el resultado de MGD.");
        }
    }

    // Actualizar la visualización de las coordenadas en la interfaz
    private void UpdateCoordinatesDisplay()
    {
        coordinatesDisplay.text = ""; // Limpiar texto anterior
        for (int i = 0; i < jointPositionsList.Count; i++)
        {
            float[] positions = jointPositionsList[i];
            string positionsString = string.Join(", ", positions);
            coordinatesDisplay.text += $"P {i + 1}: {positionsString}  Speed: {speedList[i]}";
            
            // Mostrar el delay si está activo
            if (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive())
            {
                coordinatesDisplay.text += $"  Delay: {delayList[i]}s";
            }

            // Mostrar el resultado de MGD si está disponible
            if (pointToDirectaResult.ContainsKey(i))
            {
                string mgdResult = pointToDirectaResult[i];
                coordinatesDisplay.text += $"\n    : {FormatCartesianPositions(mgdResult)}";
            }

            coordinatesDisplay.text += "\n";
        }
    }

    // Formatear las posiciones cartesianas recibidas de MGD para mejor visualización
    private string FormatCartesianPositions(string cartesianData)
    {
        string[] parts = cartesianData.Split(',');
        if (parts.Length == 6)
        {
            return $"X: {parts[0]}  Y: {parts[1]}  Z: {parts[2]}  Rx: {parts[3]}  Ry: {parts[4]}  Rz: {parts[5]}";
        }
        else
        {
            return "Datos de posición cartesiana inválidos";
        }
    }

    // Guardar las posiciones en un archivo de texto
    private void GuardarEnTxt()
    {
        string filePath = Path.Combine(savePath, "unitypositions.txt"); // Ruta completa del archivo

        using (StreamWriter writer = new StreamWriter(filePath))
        {
            // Escribir la primera línea con la palabra "articular"
            writer.WriteLine("articular");

            for (int i = 0; i < jointPositionsList.Count; i++)
            {
                // Obtener el resultado de MGD asociado al punto
                if (pointToDirectaResult.TryGetValue(i, out string mgdResult))
                {
                    // Escribir posición, velocidad y delay
                    string line = $"{mgdResult},{speedList[i]},{delayList[i]}";
                    writer.WriteLine(line);
                }
                else
                {
                    // Si no hay resultado de MGD, indicar ausencia
                    string line = $"Sin MGD,{speedList[i]},{delayList[i]}";
                    writer.WriteLine(line);
                    Debug.LogWarning($"No se encontró resultado de MGD para el punto {i}. Se guardará sin MGD.");
                }
            }
        }

        Debug.Log($"Archivo de posiciones guardado en: {filePath}");
        // No se limpian las listas para permitir agregar más posiciones si es necesario
    }

    // Enviar comandos de spline con delay al robot
    private IEnumerator SendSplineCommandsWithDelay()
    {
        if (jointPositionsList.Count == 0)
        {
            Debug.LogWarning("No hay comandos de articulaciones para enviar.");
            yield break;
        }

        // Iniciar actualizaciones de posiciones articulares
        jointPositionSubscriber.StartUpdating();

        // Enviar comandos de preparación antes de los JNTPoint()
        ros2CommandSender.SendCommand("DragTeachSwitch(0)");
        Debug.Log("Enviado: DragTeachSwitch(0)");
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("StopMotion()");
        Debug.Log("Enviado: StopMotion()");
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("ResetAllError()");
        Debug.Log("Enviado: ResetAllError()");
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("StartJOG(0,6,0,100)");
        Debug.Log("Enviado: StartJOG(0,6,0,100)");
        yield return new WaitForSeconds(0.5f);

        ros2CommandSender.SendCommand("StartJOG(0,6,1,100)");
        Debug.Log("Enviado: StartJOG(0,6,1,100)");
        yield return new WaitForSeconds(0.5f);

        // Añadir un breve delay antes de iniciar los comandos principales
        yield return new WaitForSeconds(1f);

        int batchSize = 5;
        int numBatches = Mathf.CeilToInt(jointPositionsList.Count / (float)batchSize);

        // Procesar y enviar comandos por lotes
        for (int batch = 0; batch < numBatches; batch++)
        {
            int startIdx = batch * batchSize;
            int endIdx = Mathf.Min(startIdx + batchSize, jointPositionsList.Count);

            // Enviar JNTPoint() para el lote actual
            for (int i = startIdx; i < endIdx; i++)
            {
                int localIndex = (i - startIdx) + 1; // Índice local de 1 a batchSize
                float[] jointPos = jointPositionsList[i];
                string jointPositionsString = string.Join(",", jointPos);
                string jointCommand = $"JNTPoint({localIndex},{jointPositionsString})";
                ros2CommandSender.SendCommand(jointCommand);
                Debug.Log($"Enviado: {jointCommand}");
                yield return new WaitForSeconds(0.05f); // Breve delay entre comandos
            }

            // Verificar si el modo delay está activo
            if (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive())
            {
                // Modo con delay
                for (int i = startIdx; i < endIdx; i++)
                {
                    int localIndex = (i - startIdx) + 1;
                    float currentSpeed = speedList[i];
                    string moveCommand = $"MoveJ(JNT{localIndex},{currentSpeed})";

                    ros2CommandSender.SendCommand(moveCommand);
                    Debug.Log($"Enviado comando de movimiento: {moveCommand}");

                    // Esperar a que el robot alcance la posición objetivo
                    float[] targetJointPositions = jointPositionsList[i];
                    jointPositionSubscriber.StartUpdating();
                    yield return StartCoroutine(WaitForRobotToReachPosition(targetJointPositions, 1f)); // Tolerancia de ±1 grado

                    // Esperar el delay específico del punto
                    float pointDelay = delayList[i];

                    // Excepción para el primer punto si el delay es 0
                    if (i == 0 && pointDelay == 0)
                    {
                        pointDelay = 1f; // Delay por defecto de 1 segundo
                    }

                    Debug.Log($"Posición alcanzada para JNT{localIndex}. Esperando {pointDelay} segundos antes de continuar.");
                    yield return new WaitForSeconds(pointDelay);
                }
            }
            else
            {
                // Modo sin delay
                ros2CommandSender.SendCommand("SplineStart()");
                Debug.Log("Enviado: SplineStart()");
                yield return new WaitForSeconds(0.05f);

                // Enviar comandos SplinePTP para cada punto en el lote
                for (int i = startIdx; i < endIdx; i++)
                {
                    int localIndex = (i - startIdx) + 1;
                    float currentSpeed = speedList[i];
                    string splineCommand = $"SplinePTP(JNT{localIndex},{currentSpeed})";
                    ros2CommandSender.SendCommand(splineCommand);
                    Debug.Log($"Enviado: {splineCommand}");
                    yield return new WaitForSeconds(0.05f); // Breve delay entre comandos
                }

                ros2CommandSender.SendCommand("SplineEnd()");
                Debug.Log("Enviado: SplineEnd()");
                yield return new WaitForSeconds(0.05f);
            }

            // Pausa breve entre lotes para evitar saturación
            yield return new WaitForSeconds(0.1f);
        }

        // Limpiar listas y reiniciar estados después de enviar comandos
        jointPositionsList.Clear();
        delayList.Clear();
        speedList.Clear();
        pointToDirectaResult.Clear();
        pendingMGDIndices.Clear();
        UpdateCoordinatesDisplay();

        // Reactivar actualizaciones automáticas de posiciones articulares
        isArticularModeActive = false;
        jointPositionSubscriber.StartUpdating();
    }

    // Esperar a que el robot alcance la posición objetivo dentro de una tolerancia
    private IEnumerator WaitForRobotToReachPosition(float[] targetPositions, float tolerance)
    {
        bool positionReached = false;
        float timeout = 30f; // Tiempo máximo de espera en segundos
        float elapsedTime = 0f;

        while (!positionReached && elapsedTime < timeout)
        {
            // Obtener las posiciones actuales del robot
            float[] currentPositions = jointPositionSubscriber.GetLastKnownPositions();

            if (currentPositions == null || currentPositions.Length != targetPositions.Length)
            {
                Debug.LogWarning("No se pudieron obtener las posiciones actuales del robot.");
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
                continue;
            }

            positionReached = true;
            for (int i = 0; i < targetPositions.Length; i++)
            {
                float difference = Mathf.Abs(currentPositions[i] - targetPositions[i]);
                if (difference > tolerance)
                {
                    positionReached = false;
                    break;
                }
            }

            if (!positionReached)
            {
                // Mostrar estado actual de la espera
                Debug.Log($"Esperando posición. Tiempo transcurrido: {elapsedTime}s");
                Debug.Log($"Posición actual: {string.Join(",", currentPositions)}");
                Debug.Log($"Posición objetivo: {string.Join(",", targetPositions)}");
                Debug.Log($"Diferencia: {string.Join(",", GetDifferences(currentPositions, targetPositions))}");

                // Esperar antes de volver a comprobar
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
            }
            else
            {
                // Confirmar que la posición fue alcanzada
                Debug.Log("Robot ha alcanzado la posición objetivo.");
            }
        }

        if (elapsedTime >= timeout)
        {
            Debug.LogWarning("Tiempo de espera excedido. Continuando con el siguiente comando.");
        }
    }

    // Calcular las diferencias absolutas entre posiciones actuales y objetivo
    private float[] GetDifferences(float[] current, float[] target)
    {
        float[] differences = new float[current.Length];
        for (int i = 0; i < current.Length; i++)
        {
            differences[i] = Mathf.Abs(current[i] - target[i]);
        }
        return differences;
    }
}
