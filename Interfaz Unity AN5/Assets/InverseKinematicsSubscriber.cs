/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

// Clase para suscribirse a resultados de cinemática inversa y procesarlos.
public class InverseKinematicsSubscriber : UnitySubscriber<RosString>
{
    // Referencia al script CartesianStateWriterNew para actualizar las posiciones articulares
    public CartesianStateWriterNew cartesianStateWriter;
    
    // Evento para notificar a otros scripts cuando se recibe un resultado de cinemática inversa
    public System.Action<string> OnInverseKinematicsResultReceived;

    // Inicializa la suscripción al tópico ROS correspondiente.
    protected override void Start()
    {
        base.Start();
        // Configurar el tópico al que se suscribirá
        Topic = "output_joint_position"; // Tópico que recibe el resultado de la inversa en posiciones articulares 
    }

    // Método que se llama al recibir un mensaje del tópico suscrito.
 protected override void ReceiveMessage(RosString message)
{
    Debug.Log("Recibido en output_joint_position: " + message.data);

    if (cartesianStateWriter != null)
    {
        cartesianStateWriter.ReceiveJointPositions(message.data);
    }
    else
    {
        Debug.LogError("CartesianStateWriterNew no está asignado en el Inspector.");
    }

    OnInverseKinematicsResultReceived?.Invoke(message.data);
}
}