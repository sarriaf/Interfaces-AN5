/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Collections;
using System.Collections.Generic;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class Ros2CommandSender : MonoBehaviour
{
    private RosSocket rosSocket;
    private Dictionary<string, string> advertisedTopics = new Dictionary<string, string>();

    public string commandTopic = "api_command"; // Tópico para enviar comandos a la API del robot
    public string inverseInputTopic = "input_cartesian_position"; // Tópico que envía posiciones a la cinemática inversa (cartesianas)
    public string directaInputTopic = "input_joint_position"; // Tópico que envía posiciones a la cinemática directa (articulares)

    private InputField commandInputField; // Asigna este campo en el Inspector
    private Button sendCommandButton;      // Asigna este botón en el Inspector
    public Button stopCommandButton;         // Botón original para stop
    public Button duplicateStopCommandButton; // Botón duplicado para stop

    public GameObject modeManualPanel; // Panel para modo manual, asigna en el Inspector
    public GameObject modeAutoPanel;   // Panel para modo automático, asigna en el Inspector

    private bool lastManualState; // Estado anterior del panel manual
    private bool lastAutoState;   // Estado anterior del panel automático

    void Start()
    {
        // Inicializar RosSocket con la URL del servidor ROSBridge
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketSharpProtocol("ws://localhost:9090"));

        // Asignar referencias de UI si están disponibles
        if (sendCommandButton != null && commandInputField != null)
        {
            sendCommandButton.onClick.AddListener(() => SendCommand(commandInputField.text.Trim()));
        }

        // Asignar funcionalidad al botón stop original
        if (stopCommandButton != null)
        {
            stopCommandButton.onClick.AddListener(() => SendCommand("SplineEnd()"));
            stopCommandButton.onClick.AddListener(() => SendCommand("StopMotion()"));
            stopCommandButton.onClick.AddListener(() => SendCommand("ResetAllError()"));
            stopCommandButton.onClick.AddListener(() => StartCoroutine(SendJogCommandsWithDelay()));
        }

        // Asignar la misma funcionalidad al botón stop duplicado
        if (duplicateStopCommandButton != null)
        {
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("SplineEnd()"));
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("StopMotion()"));
            duplicateStopCommandButton.onClick.AddListener(() => SendCommand("ResetAllError()"));
            duplicateStopCommandButton.onClick.AddListener(() => StartCoroutine(SendJogCommandsWithDelay()));
        }

        // Registrar el publicador para el tópico de comandos
        rosSocket.Advertise<StringMsg>(commandTopic);
        advertisedTopics[commandTopic] = commandTopic;
        Debug.Log("Conectado y tópico anunciado: " + commandTopic);

        // Registrar el publicador para el tópico de cinemática inversa
        rosSocket.Advertise<StringMsg>(inverseInputTopic);
        advertisedTopics[inverseInputTopic] = inverseInputTopic;
        Debug.Log("Tópico anunciado: " + inverseInputTopic);

        // Registrar el publicador para el tópico de cinemática directa (si es necesario)
        rosSocket.Advertise<StringMsg>(directaInputTopic);
        advertisedTopics[directaInputTopic] = directaInputTopic;
        Debug.Log("Tópico anunciado: " + directaInputTopic);

        lastManualState = modeManualPanel.activeSelf;
        lastAutoState = modeAutoPanel.activeSelf;
    }

    void Update()
    {
        // Detectar cambios en el estado del panel de modo manual
        if (modeManualPanel.activeSelf != lastManualState)
        {
            lastManualState = modeManualPanel.activeSelf;
            if (lastManualState) // Si el modo manual se activa
            {
                modeAutoPanel.SetActive(false);
                SendCommand("DragTeachSwitch(0)");
                Debug.Log("Modo manual activado, enviando comando: DragTeachSwitch(0)");
                SendCommand("SplineEnd()");
                SendCommand("ResetAllError()");
                SendCommand("StartJOG(0,6,0,100)");
                SendCommand("StartJOG(0,6,1,100)");
            }
        }

        // Detectar cambios en el estado del panel de modo automático
        if (modeAutoPanel.activeSelf != lastAutoState)
        {
            lastAutoState = modeAutoPanel.activeSelf;
            if (lastAutoState) // Si el modo automático se activa
            {
                modeManualPanel.SetActive(false);
                SendCommand("DragTeachSwitch(1)");
                Debug.Log("Modo automático activado, enviando comando: DragTeachSwitch(1)");
            }
        }
    }

    // Método para enviar un comando al tópico principal de comandos
    public void SendCommand(string command)
    {
        Debug.Log("Preparando para enviar comando: " + command);
        rosSocket.Publish(commandTopic, new StringMsg { data = command });
    }

    // Método para enviar un comando a un tópico específico
    public void SendCommandToTopic(string topic, string command)
    {
        if (!advertisedTopics.ContainsKey(topic))
        {
            rosSocket.Advertise<StringMsg>(topic);
            advertisedTopics[topic] = topic;
            Debug.Log("Tópico anunciado: " + topic);
        }
        Debug.Log("Preparando para enviar comando a " + topic + ": " + command);
        rosSocket.Publish(topic, new StringMsg { data = command });
    }

    // Corrutina para enviar comandos de JOG con retraso
    private IEnumerator SendJogCommandsWithDelay()
    {
        SendCommand("StartJOG(0,6,0,100)");
        yield return new WaitForSeconds(1.32f); // Retraso de 1.32 segundos
        SendCommand("StartJOG(0,6,1,100)");
    }

    void OnDestroy()
    {
        // Cerrar la conexión RosSocket al destruir el objeto
        if (rosSocket != null)
        {
            foreach (var topic in advertisedTopics.Values)
            {
                rosSocket.Unadvertise(topic);
                Debug.Log("Tópico desanunciado: " + topic);
            }
            rosSocket.Close();
            Debug.Log("RosSocket cerrado.");
        }
    }
}
