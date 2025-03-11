/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// Suscriptor para /current_cartesian_position, que recibe x,y,z,rx,ry,rz
/// y guarda la última posición en un array float[6]. Además, activa o desactiva
/// la señal de interpolación según se pause o reanude la lectura del tópico.
public class CartesianPositionSubscriber : UnitySubscriber<RosString>
{
    // Para pausar/reanudar la lectura de mensajes.
    private bool isUpdating = true;
    /// Indica si la interpolación debe estar activada. Cuando isUpdating es false,
    /// se activa la interpolación; cuando se reanuda, se desactiva.
    public bool InterpolationEnabled { get; private set; } = false;

    // Última posición [x, y, z, rx, ry, rz].
    private float[] lastCartesianPositions = new float[6];

    protected override void Start()
    {
        base.Start();
        Topic = "/current_cartesian_position";
        // Mensajes de log eliminados para optimizar el rendimiento.
    }
    /// Se ejecuta cada vez que llega un mensaje del tópico.
    /// Se parsean los valores y se actualiza la posición interna.
    protected override void ReceiveMessage(RosString message)
    {
        if (!isUpdating)
            return;

        string[] parts = message.data.Split(',');
        if (parts.Length != 6)
            return;

        float[] positions = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (float.TryParse(parts[i], out float val))
            {
                positions[i] = Mathf.Round(val * 100f) / 100f;
            }
            else
            {
                return;
            }
        }
        lastCartesianPositions = positions;
    }
    /// Devuelve una copia de la última posición conocida.
    public float[] GetLastKnownCartesianPositions()
    {
        return (float[])lastCartesianPositions.Clone();
    }

    /// Detiene la actualización de mensajes y activa la interpolación.
    public void StopUpdating()
    {
        isUpdating = false;
        InterpolationEnabled = true;
    }

    /// Reanuda la actualización de mensajes y desactiva la interpolación.
    public void StartUpdating()
    {
        isUpdating = true;
        InterpolationEnabled = false;
    }
}
