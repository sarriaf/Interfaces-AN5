/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.IO;
using System.Diagnostics; // Necesario para usar Process
using System.Collections;
using System.Collections.Generic;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class Sendtxt : MonoBehaviour
{
    private RosSocket rosSocket; // Conexión con ROS
    private string filePathTopic = "/input_cartesian_path";


    public Button sendHelloButton; // Botón para enviar el mensaje con la ruta del archivo
    public Button resetButton; // Botón para solucionar errores
    public Button loadTxtButton; // Botón para abrir el explorador de archivos

    private string selectedFilePath = ""; // Ruta del archivo seleccionado
    private string initialPath = "/home/tarw/Interfaz AppDesigner AN5"; // Ruta inicial para el explorador de archivos

    void Start()
    {
        // Conectar con el servidor ROSBridge
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketSharpProtocol("ws://localhost:9090"));

        // Anunciar el tópico en ROS2 para recibir la ruta del archivo
        rosSocket.Advertise<RosString>(filePathTopic);
        UnityEngine.Debug.Log("Conectado y tópico anunciado: " + filePathTopic); // Mensaje en español

        // Asignar la función al botón de cargar archivo
        if (loadTxtButton != null)
        {
            loadTxtButton.onClick.AddListener(() => StartCoroutine(OpenFileBrowser()));
        }

        // Asignar la función al botón de enviar mensaje
        if (sendHelloButton != null)
        {
            sendHelloButton.onClick.AddListener(() => PublishFilePath());
        }
    }

    // Corrutina para abrir el explorador de archivos usando Zenity
    private IEnumerator OpenFileBrowser()
    {
        string filePath = "";
        bool done = false;

        yield return new WaitForEndOfFrame(); // Esperar al final del frame para asegurar que se inicia correctamente

        // Ejecutar Zenity en un hilo separado para evitar bloquear el hilo principal
        System.Threading.Thread t = new System.Threading.Thread(() =>
        {
            // Construir el comando de Zenity para seleccionar archivos .txt
            string arguments = $"--file-selection --filename={initialPath}/ --title=\"Selecciona un archivo .txt\" --file-filter=*.txt";

            Process process = new Process();
            process.StartInfo.FileName = "zenity";
            process.StartInfo.Arguments = arguments;
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.RedirectStandardOutput = true;

            try
            {
                process.Start();

                filePath = process.StandardOutput.ReadLine(); // Leer la ruta del archivo seleccionado
                process.WaitForExit(); // Esperar a que Zenity termine
            }
            catch (System.Exception e)
            {
                UnityEngine.Debug.LogError("Error al ejecutar Zenity: " + e.Message); // Error en español
            }
            finally
            {
                done = true; // Indicar que el proceso ha finalizado
            }
        });

        t.Start(); // Iniciar el hilo

        // Esperar hasta que el hilo termine
        while (!done)
        {
            yield return null;
        }

        if (!string.IsNullOrEmpty(filePath) && File.Exists(filePath))
        {
            selectedFilePath = filePath;
            UnityEngine.Debug.Log("Archivo .txt seleccionado: " + selectedFilePath); // Mensaje en español
        }
        else
        {
            UnityEngine.Debug.Log("No se seleccionó ningún archivo."); // Mensaje en español
            selectedFilePath = ""; // Reiniciar la selección si no se seleccionó un archivo
        }
    }

    // Método que publica la ruta del archivo en el tópico
    private void PublishFilePath()
    {
        if (!string.IsNullOrEmpty(selectedFilePath))
        {
            // Crear el mensaje con la ruta del archivo
            RosString message = new RosString { data = selectedFilePath };
            rosSocket.Publish(filePathTopic, message);
            UnityEngine.Debug.Log("Ruta del archivo publicada en el tópico: " + filePathTopic); // Mensaje en español
        }
        else
        {
            UnityEngine.Debug.Log("No hay archivo seleccionado. Por favor, selecciona un archivo .txt antes de enviar."); // Mensaje en español
        }
    }

    // Cerrar la conexión al destruir el objeto
    void OnDestroy()
    {
        if (rosSocket != null)
        {
            rosSocket.Close(); // Cerrar la conexión con ROS
            UnityEngine.Debug.Log("RosSocket cerrado."); // Mensaje en español
        }
    }
}