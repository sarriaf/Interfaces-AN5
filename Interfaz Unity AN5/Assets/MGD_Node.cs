/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using UnityEngine;
using RosSharp.RosBridgeClient;
using System;
using System.Linq;
using System.Collections;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class MGD_Node : MonoBehaviour
{
    // Parámetros DH del FR5
    double[,] DH_params = new double[,] {
        {0, Math.PI / 2, 0.152, 0},
        {-0.425, 0, 0, 0},
        {-0.395, 0, 0, 0},
        {0, Math.PI / 2, 0.102, 0},
        {0, -Math.PI / 2, 0.102, 0},
        {0, 0, 0.267, 0}
    };

    private RosConnector rosConnector; // Referencia al RosConnector
    private RosSocket rosSocket; // Referencia al RosSocket

    private string inputTopic = "input_joint_position"; // topico que publica en la cinematica directa 
    private string outputTopic = "output_cartesian_position"; // topico que recibe el resultado de la cinematica directa en posciones cartesianas 

    private string inputTopicId; // ID de suscripción al tópico de entrada
    private string outputTopicId; // ID de publicador al tópico de salida

    void Start()
    {
        // Obtener la instancia de RosConnector
        rosConnector = GetComponent<RosConnector>();
        if (rosConnector == null)
        {
            Debug.LogError("RosConnector no encontrado en el mismo GameObject."); // Error en español
            return;
        }

        // Esperar a que la conexión se establezca
        StartCoroutine(WaitForConnectionAndSubscribe());
    }

    // Corrutina para esperar la conexión y suscribirse/publicar a los tópicos
    private IEnumerator WaitForConnectionAndSubscribe()
    {
        // Esperar hasta que RosConnector esté conectado
        while (!rosConnector.IsConnected.WaitOne(0))
        {
            yield return null;
        }

        // Obtener el RosSocket una vez conectado
        rosSocket = rosConnector.RosSocket;

        // Suscribirse al tópico input_joint_position
        inputTopicId = rosSocket.Subscribe<StringMsg>(
            inputTopic,
            ListenerCallback,
            queue_length: 1);

        // Registrar el publicador para output_cartesian_position
        outputTopicId = rosSocket.Advertise<StringMsg>(outputTopic);

        Debug.Log("Suscrito al tópico: " + inputTopic); // Mensaje en español
        Debug.Log("Publicador registrado para el tópico: " + outputTopic); // Mensaje en español
    }

    // Callback que se ejecuta al recibir un mensaje del tópico suscrito
    void ListenerCallback(StringMsg message)
    {
        // Convertir la cadena de texto a una lista de ángulos
        string[] angleStrings = message.data.Split(',');
        double[] theta_deg;

        try
        {
            theta_deg = Array.ConvertAll(angleStrings, Double.Parse);
        }
        catch (FormatException ex)
        {
            Debug.LogError("Error al convertir los ángulos: " + ex.Message); // Error en español
            return;
        }

        // Convertir los ángulos de grados a radianes
        double[] theta = theta_deg.Select(angle => angle * Math.PI / 180).ToArray();

        // Inicializar la matriz de transformación total como identidad
        double[,] T_total = MatrixIdentity(4);

        // Calcular la matriz de transformación para cada articulación
        for (int i = 0; i < theta.Length; i++)
        {
            double a = DH_params[i, 0];
            double alpha = DH_params[i, 1];
            double d = DH_params[i, 2];
            double th = theta[i] + DH_params[i, 3];

            double[,] T_i = dh_matrix(a, alpha, d, th);
            T_total = MatrixMultiply(T_total, T_i);
        }

        // Extraer la posición del efector final y convertir a milímetros
        double Px = Truncate(T_total[0, 3] * 1000, 2);
        double Py = Truncate(T_total[1, 3] * 1000, 2);
        double Pz = Truncate(T_total[2, 3] * 1000, 2);

        // Extraer la matriz de rotación 3x3 del efector final
        double[,] rotation_matrix = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rotation_matrix[i, j] = T_total[i, j];

        // Calcular rx, ry, rz usando la matriz de rotación y convertir a grados
        double ry = Truncate(Math.Atan2(-rotation_matrix[2, 0],
            Math.Sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])) * 180 / Math.PI, 2);
        double rx = Truncate(Math.Atan2(rotation_matrix[2, 1], rotation_matrix[2, 2]) * 180 / Math.PI, 2);
        double rz = Truncate(Math.Atan2(rotation_matrix[1, 0], rotation_matrix[0, 0]) * 180 / Math.PI, 2);

        // Crear la cadena de texto de salida con la posición y los ángulos de rotación
        string salida = $"{Px},{Py},{Pz},{rx},{ry},{rz}";

        // Publicar la salida en output_cartesian_position
        StringMsg outputMsg = new StringMsg();
        outputMsg.data = salida;
        rosSocket.Publish(outputTopicId, outputMsg);

        // Mostrar la salida en consola
        Debug.Log($"Salida [Px, Py, Pz, rx, ry, rz]: {salida}"); // Mensaje en español
    }

    // Función para calcular la matriz de transformación de Denavit-Hartenberg
    double[,] dh_matrix(double a, double alpha, double d, double theta)
    {
        double[,] matrix = new double[4, 4];

        matrix[0, 0] = Math.Cos(theta);
        matrix[0, 1] = -Math.Sin(theta) * Math.Cos(alpha);
        matrix[0, 2] = Math.Sin(theta) * Math.Sin(alpha);
        matrix[0, 3] = a * Math.Cos(theta);

        matrix[1, 0] = Math.Sin(theta);
        matrix[1, 1] = Math.Cos(theta) * Math.Cos(alpha);
        matrix[1, 2] = -Math.Cos(theta) * Math.Sin(alpha);
        matrix[1, 3] = a * Math.Sin(theta);

        matrix[2, 0] = 0;
        matrix[2, 1] = Math.Sin(alpha);
        matrix[2, 2] = Math.Cos(alpha);
        matrix[2, 3] = d;

        matrix[3, 0] = 0;
        matrix[3, 1] = 0;
        matrix[3, 2] = 0;
        matrix[3, 3] = 1;

        return matrix;
    }

    // Función para crear una matriz identidad de tamaño n x n
    double[,] MatrixIdentity(int n)
    {
        double[,] identity = new double[n, n];
        for (int i = 0; i < n; i++)
            identity[i, i] = 1;
        return identity;
    }

    // Función para multiplicar dos matrices
    double[,] MatrixMultiply(double[,] A, double[,] B)
    {
        int rowsA = A.GetLength(0);
        int colsA = A.GetLength(1);
        int colsB = B.GetLength(1);
        double[,] result = new double[rowsA, colsB];

        for (int i = 0; i < rowsA; i++)
            for (int j = 0; j < colsB; j++)
            {
                double sum = 0;
                for (int k = 0; k < colsA; k++)
                    sum += A[i, k] * B[k, j];
                result[i, j] = sum;
            }
        return result;
    }

    // Función para truncar números a un número específico de decimales sin redondear
    double Truncate(double number, int decimals)
    {
        double factor = Math.Pow(10, decimals);
        return Math.Floor(number * factor) / factor;
    }

    void OnDestroy()
    {
        // Cancelar suscripciones y anuncios
        if (rosSocket != null)
        {
            if (!string.IsNullOrEmpty(inputTopicId))
                rosSocket.Unsubscribe(inputTopicId);
            if (!string.IsNullOrEmpty(outputTopicId))
                rosSocket.Unadvertise(outputTopicId);
        }
    }
}
