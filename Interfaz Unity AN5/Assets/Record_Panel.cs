/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using System.Collections;
using System.Collections.Generic;
using System.IO; // Necesario para escribir archivos
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

public class recordPanel : MonoBehaviour
{
    // Botones de interfaz
    public Button grabarButton;      // Botón para grabar posiciones actuales
    public Button enviarButton;      // Botón para enviar todas las posiciones grabadas
    public Button clearButton;       // Botón para limpiar (eliminar el último comando grabado)
    public Button saveTxtButton;     // Botón para guardar las posiciones en un archivo txt
    public Text grabarDisplay;       // Texto donde se muestran los puntos añadidos

    // Suscriptores y enviadores de comandos ROS
    public Ros2CommandSender ros2CommandSender;                    // Para enviar comandos ROS
    public JointPositionSubscriber jointPositionSubscriber;        // Suscripción a posiciones articulares
    public CartesianPositionSubscriber cartesianPositionSubscriber; // Suscripción a posiciones cartesianas

    // Controles de velocidad
    public Slider speedSlider;           // Slider para controlar la velocidad de movimiento
    public InputField speedInputField;   // Campo de entrada para mostrar y cambiar la velocidad
    public Button increaseSpeedButton;   // Botón para aumentar la velocidad
    public Button decreaseSpeedButton;   // Botón para disminuir la velocidad

    // Listas para almacenar comandos y posiciones
    private List<string> jointCommands = new List<string>();         // Lista para almacenar comandos JNTPoint
    private List<float[]> jointPositionsList = new List<float[]>();    // Lista para almacenar las posiciones articulares
    private List<float[]> cartesianPositionsList = new List<float[]>();  // Lista para almacenar las posiciones cartesianas
    private List<int> pointIndices = new List<int>();                  // Índices de los puntos grabados
    private List<float> speedList = new List<float>();                 // Lista para almacenar las velocidades asociadas a cada punto
    private List<float> delayList = new List<float>();                 // Lista para almacenar los delays de cada punto

    // Variables internas
    private int pointIndex = 1;                  // Índice para los comandos JNTPoint
    private float[] jointPositions = new float[6];        // Posiciones articulares actuales
    private float[] lastJointPositions = new float[6];    // Última posición de articulaciones para evitar duplicados
    private float[] lastCartesianPositions = new float[6]; // Última posición cartesiana para evitar duplicados
    private float speed = 10f;                    // Velocidad predeterminada
    private bool isFirstPoint = true;             // Verificar si es el primer punto grabado
    private string savePath = "/home/tarw/Interfaz AppDesigner AN5"; // Ruta donde se guardará el archivo txt

    void Start()
    {
        // Asignar listeners a los botones de la interfaz
        grabarButton.onClick.AddListener(GrabarPosicionActual); 
        enviarButton.onClick.AddListener(() => StartCoroutine(EnviarComandosConDelay())); 
        clearButton.onClick.AddListener(LimpiarComandos); 
        saveTxtButton.onClick.AddListener(GuardarEnTxt);

        // Configuración del Slider y el InputField para la velocidad
        if (speedSlider != null)
        {
            speedSlider.minValue = 1f;
            speedSlider.maxValue = 100f;
            speedSlider.wholeNumbers = true; // Asegurar que solo se manejen valores enteros
            speedSlider.value = speed;
            speedSlider.onValueChanged.AddListener(OnSpeedSliderChanged);
        }

        if (speedInputField != null)
        {
            speedInputField.text = speed.ToString();
            speedInputField.onEndEdit.AddListener(OnSpeedInputFieldChanged);
        }

        // Configurar eventos para los botones de aumentar y disminuir la velocidad
        if (increaseSpeedButton != null)
        {
            increaseSpeedButton.onClick.AddListener(IncreaseSpeedValue);
        }

        if (decreaseSpeedButton != null)
        {
            decreaseSpeedButton.onClick.AddListener(DecreaseSpeedValue);
        }

        // Iniciar la actualización de posiciones articulares por defecto
        if (jointPositionSubscriber != null)
        {
            jointPositionSubscriber.StartUpdating();
        }
        else
        {
            Debug.LogError("JointPositionSubscriber no está asignado en el Inspector.");
        }
    }

    // Método para grabar la posición actual del robot
    private void GrabarPosicionActual()
    {
        // Obtener las últimas posiciones articulares y cartesianas conocidas directamente de los suscriptores
        jointPositions = jointPositionSubscriber.GetLastKnownPositions();
        float[] cartesianPositions = cartesianPositionSubscriber.GetLastKnownCartesianPositions();

        if (isFirstPoint)
        {
            // Si es el primer punto, guardarlo directamente sin comparación
            GuardarPunto(cartesianPositions);
            isFirstPoint = false; // Ya no es el primer punto
        }
        else
        {
            // Verificar si ha habido un cambio significativo en las posiciones
            bool haCambiado = false;
            for (int i = 0; i < jointPositions.Length; i++)
            {
                if (Mathf.Abs(jointPositions[i] - lastJointPositions[i]) > 0.01f ||
                    Mathf.Abs(cartesianPositions[i] - lastCartesianPositions[i]) > 0.1f) // Cambios mayores a 0.01 para articulares y 0.1 para cartesianas
                {
                    haCambiado = true;
                    break;
                }
            }

            if (haCambiado)
            {
                // Si ha habido cambios, guardar el nuevo punto
                GuardarPunto(cartesianPositions);
            }
            else
            {
                Debug.LogWarning("La posición no ha cambiado. No se grabará un nuevo punto.");
            }
        }
    }

    // Método para guardar la posición actual y actualizar la visualización
    private void GuardarPunto(float[] cartesianPositions)
    {
        // Crear y guardar el comando JNTPoint en la lista
        string jointCommand = $"JNTPoint({pointIndex},{jointPositions[0]},{jointPositions[1]},{jointPositions[2]},{jointPositions[3]},{jointPositions[4]},{jointPositions[5]})";
        jointCommands.Add(jointCommand); 
        jointPositionsList.Add((float[])jointPositions.Clone()); // Guardar la posición articular actual en la lista
        pointIndices.Add(pointIndex);    
        speedList.Add(speed);            

        // Obtener el delay asociado al punto si el modo delay está activo
        float pointDelay = 0f;
        if (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive())
        {
            pointDelay = DelayModeController.Instance.GetDelayTime();
        }
        delayList.Add(pointDelay);       

        pointIndex++;                    

        // Actualizar el display de grabación con la información del nuevo punto
        grabarDisplay.text += $"P {pointIndex - 1}: " +
                              $" {jointPositions[0]},  {jointPositions[1]},  {jointPositions[2]}, " +
                              $" {jointPositions[3]},  {jointPositions[4]},  {jointPositions[5]}, Speed: {speed}";

        // Mostrar el delay si el modo delay está activo
        if (DelayModeController.Instance != null && DelayModeController.Instance.IsDelayModeActive())
        {
            grabarDisplay.text += $", Delay: {pointDelay}s";
        }

        // Añadimos el salto de línea al final de este nuevo punto
        grabarDisplay.text += "\n"; 

        // Actualizar las últimas posiciones grabadas
        jointPositions.CopyTo(lastJointPositions, 0);
        cartesianPositions.CopyTo(lastCartesianPositions, 0);

        // Guardar la posición cartesiana actual en la lista
        cartesianPositionsList.Add((float[])cartesianPositions.Clone());

        Debug.Log($"Posición cartesiana añadida a la lista: {string.Join(",", cartesianPositions)}");
    }

    // Corrutina para enviar los comandos grabados con delay basado en la posición alcanzada
    private IEnumerator EnviarComandosConDelay()
    {
        if (jointCommands.Count == 0)
        {
            Debug.LogWarning("No hay comandos de articulaciones grabados para enviar.");
            yield break;
        }

        // Asegurar que jointPositionSubscriber está actualizando
        if (jointPositionSubscriber != null)
        {
            jointPositionSubscriber.StartUpdating();
        }
        else
        {
            Debug.LogError("JointPositionSubscriber no está asignado en el Inspector.");
            yield break;
        }

        // Enviar los comandos de preparación antes de los JNTPoint()
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
        yield return new WaitForSeconds(0.05f);

        ros2CommandSender.SendCommand("StartJOG(0,6,1,100)");
        Debug.Log("Enviado: StartJOG(0,6,1,100)");
        yield return new WaitForSeconds(0.05f);

        // Añadir un delay mínimo antes de enviar el primer comando
        yield return new WaitForSeconds(2f);

        int batchSize = 5;
        int numBatches = Mathf.CeilToInt(jointCommands.Count / (float)batchSize);

        for (int batch = 0; batch < numBatches; batch++)
        {
            int startIdx = batch * batchSize;
            int endIdx = Mathf.Min(startIdx + batchSize, jointCommands.Count);

            // Enviar los comandos JNTPoint() para definir los puntos articulares en el lote actual
            for (int i = startIdx; i < endIdx; i++)
            {
                int localIndex = (i - startIdx) + 1; // Índice local de 1 a batchSize

                // Modificar el comando JNTPoint para usar el localIndex
                string jointCommand = jointCommands[i];
                string modifiedJointCommand = jointCommand.Replace($"JNTPoint({pointIndices[i]}", $"JNTPoint({localIndex}");

                ros2CommandSender.SendCommand(modifiedJointCommand);
                Debug.Log($"Enviado: {modifiedJointCommand}");
                yield return new WaitForSeconds(0.05f); // Pequeño delay entre comandos
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

                    // Esperar hasta que el robot alcance la posición objetivo
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
                // Enviar el comando SplineStart
                ros2CommandSender.SendCommand("SplineStart()");
                Debug.Log("Enviado: SplineStart()");
                yield return new WaitForSeconds(0.05f);

                // Enviar los comandos SplinePTP para cada punto añadido con su velocidad específica
                for (int i = startIdx; i < endIdx; i++)
                {
                    int localIndex = (i - startIdx) + 1;
                    float currentSpeed = speedList[i];
                    string splineCommand = $"SplinePTP(JNT{localIndex},{currentSpeed})"; // Usar la velocidad correspondiente
                    ros2CommandSender.SendCommand(splineCommand);
                    Debug.Log($"Enviado: {splineCommand}");
                    yield return new WaitForSeconds(0.05f); // Pequeño delay entre comandos
                }

                // Enviar el comando SplineEnd
                ros2CommandSender.SendCommand("SplineEnd()");
                Debug.Log("Enviado: SplineEnd()");
            }

            // Pausa breve entre lotes para evitar saturación
            yield return new WaitForSeconds(0.1f);
        }

        // **********************************************************************
        // Limpieza de todos los vectores y el texto tras el envío
        // **********************************************************************
        LimpiarTodo();
    }

    // Método para limpiar todo después de enviar
    private void LimpiarTodo()
    {
        jointCommands.Clear();
        jointPositionsList.Clear();
        cartesianPositionsList.Clear();
        pointIndices.Clear();
        speedList.Clear();
        delayList.Clear();
        
        // Resetear índices y estado del primer punto
        pointIndex = 1;
        isFirstPoint = true;

        // Limpiar el grabarDisplay
        grabarDisplay.text = "";

        Debug.Log("Se han vaciado todos los vectores y limpiado el grabarDisplay tras el envío.");
    }

    // Corrutina para esperar a que el robot alcance la posición objetivo
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
                // Imprimir las posiciones actuales y objetivo
                Debug.Log($"Esperando posición. Tiempo transcurrido: {elapsedTime}s");
                Debug.Log($"Posición actual: {string.Join(",", currentPositions)}");
                Debug.Log($"Posición objetivo: {string.Join(",", targetPositions)}");
                Debug.Log($"Diferencia: {string.Join(",", GetDifferences(currentPositions, targetPositions))}");

                // Esperar un breve momento antes de volver a comprobar
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
            }
            else
            {
                // Posición alcanzada
                Debug.Log("Robot ha alcanzado la posición objetivo.");
            }
        }

        if (elapsedTime >= timeout)
        {
            Debug.LogWarning("Tiempo de espera excedido. Continuando con el siguiente comando.");
        }
    }

    // Método auxiliar para calcular las diferencias absolutas entre posiciones
    private float[] GetDifferences(float[] current, float[] target)
    {
        float[] differences = new float[current.Length];
        for (int i = 0; i < current.Length; i++)
        {
            differences[i] = Mathf.Abs(current[i] - target[i]);
        }
        return differences;
    }

    // Método para eliminar (limpiar) el último comando/punto grabado
    private void LimpiarComandos()
    {
        if (jointCommands.Count > 0)
        {
            // Eliminar el último elemento de cada lista asociada al punto
            jointCommands.RemoveAt(jointCommands.Count - 1);
            jointPositionsList.RemoveAt(jointPositionsList.Count - 1);
            cartesianPositionsList.RemoveAt(cartesianPositionsList.Count - 1);
            pointIndices.RemoveAt(pointIndices.Count - 1);
            speedList.RemoveAt(speedList.Count - 1);
            delayList.RemoveAt(delayList.Count - 1);

            // Decrementar el índice de puntos
            pointIndex--;

            // Si ya no hay puntos grabados, reiniciar indicador y limpiar el display
            if (jointCommands.Count == 0)
            {
                isFirstPoint = true;
                grabarDisplay.text = "";
            }
            else
            {
                // Actualizar el display: se elimina la última línea agregada
                string[] lines = grabarDisplay.text.Split(new[] { '\n' }, System.StringSplitOptions.None);
                if (lines.Length > 0)
                {
                    List<string> listLines = new List<string>(lines);

                    // Quitar líneas vacías finales (si las hubiera)
                    while (listLines.Count > 0 && string.IsNullOrEmpty(listLines[listLines.Count - 1]))
                    {
                        listLines.RemoveAt(listLines.Count - 1);
                    }

                    // Remover la última línea real de texto
                    if (listLines.Count > 0)
                    {
                        listLines.RemoveAt(listLines.Count - 1);
                    }

                    // IMPORTANTE: si aún queda texto, forzamos un salto de línea al final
                    if (listLines.Count > 0)
                    {
                        grabarDisplay.text = string.Join("\n", listLines) + "\n";
                    }
                    else
                    {
                        grabarDisplay.text = "";
                    }
                }
            }
            Debug.Log("Último punto eliminado.");
        }
        else
        {
            Debug.LogWarning("No hay puntos para eliminar.");
        }
    }

    // Evento cuando se cambia el valor del slider de velocidad
    private void OnSpeedSliderChanged(float value)
    {
        speed = Mathf.Round(value); // Aproximar la velocidad al entero más cercano
        speedSlider.value = speed; // Asegurar que el slider muestra el valor redondeado
        if (speedInputField != null)
        {
            speedInputField.text = speed.ToString(); // Actualizar el valor en el input field
        }
        Debug.Log($"Velocidad ajustada a: {speed}");
    }

    // Evento cuando se cambia el valor en el InputField de velocidad
    private void OnSpeedInputFieldChanged(string value)
    {
        if (float.TryParse(value, out float newSpeed))
        {
            newSpeed = Mathf.Round(newSpeed); // Aproximar al entero más cercano
            newSpeed = Mathf.Clamp(newSpeed, speedSlider.minValue, speedSlider.maxValue); // Limitar la velocidad al rango permitido
            speed = newSpeed; // Actualizar la velocidad
            speedSlider.value = newSpeed; // Asegurar que el slider se actualice
            Debug.Log($"Velocidad configurada manualmente a: {speed}");
        }
        else
        {
            speedInputField.text = speed.ToString(); // Si el valor ingresado no es válido, restaurar el anterior
            Debug.LogWarning("Entrada inválida para la velocidad. Se restauró el valor anterior.");
        }
    }

    // Método para aumentar la velocidad en 1 unidad
    private void IncreaseSpeedValue()
    {
        speed = Mathf.Clamp(speed + 1f, speedSlider.minValue, speedSlider.maxValue);
        speedSlider.value = speed;
        if (speedInputField != null)
        {
            speedInputField.text = speed.ToString();
        }
        Debug.Log($"Velocidad incrementada a: {speed}");
    }

    // Método para disminuir la velocidad en 1 unidad
    private void DecreaseSpeedValue()
    {
        speed = Mathf.Clamp(speed - 1f, speedSlider.minValue, speedSlider.maxValue);
        speedSlider.value = speed;
        if (speedInputField != null)
        {
            speedInputField.text = speed.ToString();
        }
        Debug.Log($"Velocidad decrementada a: {speed}");
    }

    // Método para guardar las posiciones cartesianas en un archivo txt
    private void GuardarEnTxt()
    {
        string filePath = Path.Combine(savePath, "record_positions.txt"); // Definir la ruta completa del archivo

        try
        {
            using (StreamWriter writer = new StreamWriter(filePath))
            {
                // Escribir la primera línea con la palabra "cartesiano"
                writer.WriteLine("cartesiano");
                
                for (int i = 0; i < cartesianPositionsList.Count; i++)
                {
                    // Incluir la velocidad y el delay como parámetros adicionales en la línea de salida
                    string line = string.Join(",", cartesianPositionsList[i]) + $",{speedList[i]},{delayList[i]}"; 
                    writer.WriteLine(line); // Escribir cada línea en el archivo
                }
            }

            Debug.Log($"Archivo guardado en: {filePath}");
            // No se limpian las listas aquí para permitir agregar más posiciones si es necesario
        }
        catch (IOException ex)
        {
            Debug.LogError($"Error al guardar el archivo: {ex.Message}");
        }
    }
}
