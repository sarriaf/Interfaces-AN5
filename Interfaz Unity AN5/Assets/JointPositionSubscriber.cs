/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/

using System;
using UnityEngine;
using RosSharp.RosBridgeClient;
// Alias para el tipo de mensaje ROS estándar String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class JointPositionSubscriber : UnitySubscriber<RosString>
{
    // Array de objetos encargados de escribir y actualizar el estado de las articulaciones en la representación visual.
    public JointStateWriter[] jointStateWriters;

    // Flag que determina si se deben procesar y actualizar las posiciones recibidas.
    private bool isUpdating = true;

    // Evento que notifica a otros componentes cuando se actualizan las posiciones de las articulaciones.
    public event Action<float[]> OnJointPositionsUpdated;

    // Almacena la última posición conocida de cada articulación.
    private float[] lastPositions;

    // Configuración para el modo "freeze" que ignora cambios pequeños.
    [Header("Modo Freeze (para ignorar pequeños cambios)")]
    [Tooltip("Si está activo, solo actualiza el URDF cuando la diferencia > freezeThresholdDeg en alguna articulación.")]
    public bool freezeMode = false;

    [Tooltip("Umbral de diferencia en grados para salir del freeze en una articulación (ej. 0.5)")]
    public float freezeThresholdDeg = 0.5f;

    // Método de inicialización del componente.
    protected override void Start()
    {
        // Se invoca el método Start de la clase base para inicializar la suscripción.
        base.Start();

        // Se inicializa el arreglo lastPositions según la cantidad de jointStateWriters asignados.
        if (jointStateWriters != null && jointStateWriters.Length > 0)
            lastPositions = new float[jointStateWriters.Length];
        else
            // Si no se han asignado escritores, se asume un arreglo de 6 articulaciones por defecto.
            lastPositions = new float[6];

        // Se define el tópico de ROS al que se suscribirá este componente.
        Topic = "current_joint_position";
    }

    // Método que procesa los mensajes recibidos desde ROS.
    protected override void ReceiveMessage(RosString message)
    {
        // Si la actualización está deshabilitada, se ignora el mensaje.
        if (!isUpdating)
            return;

        // Se separa el mensaje recibido utilizando la coma como delimitador.
        string[] parts = message.data.Split(',');
        // Se verifica que la cantidad de valores coincida con el número de articulaciones a actualizar.
        if (parts.Length != jointStateWriters.Length)
            return;

        // Se parsean y almacenan las nuevas posiciones de las articulaciones.
        float[] newPositions = new float[jointStateWriters.Length];
        for (int i = 0; i < jointStateWriters.Length; i++)
        {
            // Si no se puede convertir el valor, se abandona la actualización.
            if (!float.TryParse(parts[i], out float degValue))
                return;
            // Se redondea el valor a dos decimales para evitar actualizaciones con fluctuaciones insignificantes.
            newPositions[i] = Mathf.Round(degValue * 100f) / 100f;
        }

        // Se calcula la diferencia máxima entre las nuevas posiciones y las posiciones almacenadas previamente.
        float computedMaxDiff = 0f;
        for (int i = 0; i < newPositions.Length; i++)
        {
            float diff = Mathf.Abs(newPositions[i] - lastPositions[i]);
            if (diff > computedMaxDiff)
                computedMaxDiff = diff;
        }

        // Si la diferencia máxima es mayor o igual a 0.5 grados, se actualizan los escritores sin interpolación.
        if (computedMaxDiff >= 0.5f)
        {
            // Se deshabilita la interpolación y se desbloquea la escritura en cada writer.
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = false;
                writer.UnlockWriting();
            }
            // Se escribe la nueva posición (convertida a radianes) para cada articulación.
            for (int i = 0; i < jointStateWriters.Length; i++)
            {
                float jointRad = newPositions[i] * Mathf.Deg2Rad;
                jointStateWriters[i].Write(jointRad);
            }
            // Se actualiza el arreglo lastPositions con las nuevas posiciones.
            lastPositions = newPositions;
        }
        // Si el modo freeze está activado y la diferencia es menor que el umbral definido,
        // se notifica la actualización sin modificar la representación visual.
        else if (freezeMode && computedMaxDiff < freezeThresholdDeg)
        {
            OnJointPositionsUpdated?.Invoke(newPositions);
            return;
        }
        else
        {
            // En cualquier otro caso, se habilita la interpolación y se desbloquea la escritura.
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = true;
                writer.UnlockWriting();
            }
        }
        // Se invoca el evento para notificar a los suscriptores de la actualización de posiciones.
        OnJointPositionsUpdated?.Invoke(newPositions);
    }

    // Retorna una copia de las últimas posiciones conocidas de las articulaciones.
    public float[] GetLastKnownPositions()
    {
        return (float[])lastPositions.Clone();
    }

    // Desactiva la actualización de posiciones, ignorando futuros mensajes.
    public void StopUpdating()
    {
        isUpdating = false;
    }

    // Activa la actualización de posiciones para procesar nuevos mensajes.
    public void StartUpdating()
    {
        isUpdating = true;
    }
}
