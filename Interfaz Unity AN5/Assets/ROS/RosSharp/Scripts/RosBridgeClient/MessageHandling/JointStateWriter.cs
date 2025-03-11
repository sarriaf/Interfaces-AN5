/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/

using UnityEngine;
using RosSharp.Urdf;
using System.Collections;

public class JointStateWriter : MonoBehaviour
{
    // Nombre de la articulación (por ejemplo, "j1_Link", "j2_Link", etc.)
    [Header("Nombre de la articulación (ej. j1_Link, j2_Link, etc.)")]
    public string JointName;

    // Configuración de interpolación: duración en segundos para interpolar entre la posición actual y el ángulo objetivo.
    [Header("Interpolación")]
    [Tooltip("Tiempo (en segundos) para interpolar entre la posición actual y el ángulo objetivo.")]
    public float lerpDuration = 1.0f;

    // Propiedad que permite activar o desactivar la interpolación.
    public bool InterpolationEnabled { get; set; } = false;

    // --- Variables internas ---
    // Referencia al componente UrdfJoint, utilizado para actualizar la rotación en el URDF.
    private UrdfJoint urdfJoint;
    // Almacena la rotación inicial del objeto (offset base) obtenido en Start().
    private Vector3 initialRotation;
    // Nuevo estado recibido (en radianes) para la articulación.
    private float newState;
    // Indica si se ha recibido un nuevo estado que requiere actualización.
    private bool isNewStateReceived = false;
    // Referencia a la coroutine de interpolación en ejecución, si existe.
    private Coroutine interpolationCoroutine = null;
    // Almacena el ángulo actual (en grados) de la articulación.
    private float currentAngle;
    // Bloquea nuevas actualizaciones mientras se está realizando una interpolación.
    private bool lockUpdates = false;

    // Método de inicialización del componente.
    private void Start()
    {
        // Se obtiene el componente UrdfJoint del GameObject.
        urdfJoint = GetComponent<UrdfJoint>();
        if (urdfJoint == null)
            return; // Si no se encuentra el componente, se detiene la ejecución.

        // Se almacena la rotación inicial del objeto.
        initialRotation = urdfJoint.transform.localEulerAngles;
        // Se obtiene el ángulo actual de la articulación basado en la transformación.
        currentAngle = GetCurrentAngleFromTransform();
    }

    // Se ejecuta en cada frame.
    private void Update()
    {
        // Si se ha recibido un nuevo estado...
        if (isNewStateReceived)
        {
            // ... y la interpolación está habilitada, se inicia la coroutine de interpolación si no está ya en ejecución.
            if (InterpolationEnabled)
            {
                if (interpolationCoroutine == null)
                    interpolationCoroutine = StartCoroutine(InterpolateRotation());
            }
            // Si la interpolación no está activada, se aplica la rotación objetivo de forma inmediata.
            else
            {
                ApplyTargetRotation();
            }
            // Se reinicia la bandera para indicar que el nuevo estado ya fue procesado.
            isNewStateReceived = false;
        }
    }

    // Recibe el nuevo estado (en radianes) para la articulación.  
    // Si la escritura está bloqueada o ya se está interpolando, se ignoran nuevos valores.
    public void Write(float state)
    {
        if (urdfJoint == null)
            return; // No se puede proceder sin el componente UrdfJoint.
        if (lockUpdates)
            return; // Si las actualizaciones están bloqueadas, se ignora el nuevo valor.
        if (InterpolationEnabled && interpolationCoroutine != null)
            return; // Si ya se está interpolando, se ignora el nuevo estado.

        // Se asigna el nuevo estado y se marca que se recibió un nuevo valor.
        newState = state;
        isNewStateReceived = true;
    }

    // Permite desbloquear la escritura para aceptar nuevas actualizaciones (por ejemplo, al presionar "Send").
    public void UnlockWriting()
    {
        lockUpdates = false;
    }

    // Coroutine que interpola la rotación desde la posición actual hasta la posición objetivo.
    private IEnumerator InterpolateRotation()
    {
        // Convierte el nuevo estado de radianes a grados y lo invierte.
        float targetValue = newState * Mathf.Rad2Deg * -1f;
        // Se inicia la rotación objetivo utilizando la rotación inicial como base.
        Vector3 targetRotation = initialRotation;

        // Determina la rotación objetivo según el nombre de la articulación.
        switch (JointName)
        {
            case "j1_Link":
                targetRotation.x = 0;
                targetRotation.y = targetValue;
                targetRotation.z = 0;
                break;
            case "j2_Link":
                targetRotation.x = targetValue;
                targetRotation.y = 0;
                targetRotation.z = -90;
                break;
            case "j3_Link":
                targetRotation.x = 0;
                targetRotation.y = targetValue;
                targetRotation.z = 0;
                break;
            case "j4_Link":
                targetRotation.x = initialRotation.x;
                targetRotation.y = targetValue;
                targetRotation.z = initialRotation.z;
                break;
            case "j5_Link":
                targetRotation.x = targetValue;
                targetRotation.y = 0;
                targetRotation.z = -90;
                break;
            case "j6_Link":
                targetRotation.x = targetValue;
                targetRotation.y = 0;
                targetRotation.z = 90;
                break;
            default:
                yield break; // Si no coincide con ningún caso, se sale de la coroutine.
        }

        // Normaliza los ángulos de la rotación objetivo para mantenerlos en el rango [-180, 180].
        targetRotation.x = NormalizeAngle(targetRotation.x);
        targetRotation.y = NormalizeAngle(targetRotation.y);
        targetRotation.z = NormalizeAngle(targetRotation.z);

        // Obtiene la rotación actual y la normaliza.
        Vector3 startRotation = urdfJoint.transform.localEulerAngles;
        startRotation.x = NormalizeAngle(startRotation.x);
        startRotation.y = NormalizeAngle(startRotation.y);
        startRotation.z = NormalizeAngle(startRotation.z);

        // Forzar valores fijos en la rotación de inicio según el nombre de la articulación.
        if (JointName == "j2_Link")
        {
            startRotation.y = 0;
            startRotation.z = -90;
        }
        else if (JointName == "j4_Link")
        {
            startRotation.x = initialRotation.x;
            startRotation.z = initialRotation.z;
        }
        else if (JointName == "j5_Link")
        {
            startRotation.y = 0;
            startRotation.z = -90;
        }
        else if (JointName == "j6_Link")
        {
            startRotation.y = 0;
            startRotation.z = 90;
        }

        // Interpola gradualmente desde la rotación de inicio hasta la rotación objetivo durante 'lerpDuration' segundos.
        float elapsed = 0f;
        while (elapsed < lerpDuration)
        {
            elapsed += Time.deltaTime;
            // Calcula el factor de interpolación (valor entre 0 y 1).
            float t = Mathf.Clamp01(elapsed / lerpDuration);
            // Calcula la nueva rotación para cada eje utilizando DeltaAngle para obtener la diferencia angular correcta.
            Vector3 newRot = new Vector3(
                startRotation.x + Mathf.DeltaAngle(startRotation.x, targetRotation.x) * t,
                startRotation.y + Mathf.DeltaAngle(startRotation.y, targetRotation.y) * t,
                startRotation.z + Mathf.DeltaAngle(startRotation.z, targetRotation.z) * t
            );
            // Aplica la nueva rotación al objeto.
            urdfJoint.transform.localEulerAngles = newRot;
            yield return null; // Espera al siguiente frame.
        }
        // Asegura que la rotación final sea exactamente la rotación objetivo.
        urdfJoint.transform.localEulerAngles = targetRotation;
        // Actualiza el ángulo actual con el valor objetivo.
        currentAngle = targetValue;
        // Reinicia la referencia a la coroutine.
        interpolationCoroutine = null;
        // Libera el bloqueo de actualizaciones.
        lockUpdates = false;
        // Desactiva la interpolación.
        InterpolationEnabled = false;
    }

    // Aplica inmediatamente la rotación objetivo sin realizar interpolación.
    // Se respetan ciertos ejes fijos para algunas articulaciones (por ejemplo, j4_Link, j5_Link y j6_Link).
    private void ApplyTargetRotation()
    {
        // Convierte el nuevo estado a grados e invierte el valor.
        float targetValue = newState * Mathf.Rad2Deg * -1f;
        // Inicializa la rotación objetivo usando la rotación inicial.
        Vector3 targetRotation = initialRotation;
        // Determina la rotación objetivo según el nombre de la articulación.
        switch (JointName)
        {
            case "j1_Link":
                targetRotation.x = 0;
                targetRotation.y = targetValue;
                targetRotation.z = 0;
                break;
            case "j2_Link":
                targetRotation.x = targetValue;
                targetRotation.y = 0;
                targetRotation.z = -90;
                break;
            case "j3_Link":
                targetRotation.x = 0;
                targetRotation.y = targetValue;
                targetRotation.z = 0;
                break;
            case "j4_Link":
                targetRotation.x = initialRotation.x;
                targetRotation.y = targetValue;
                targetRotation.z = initialRotation.z;
                break;
            case "j5_Link":
                targetRotation.x = targetValue;
                targetRotation.y = 0;
                targetRotation.z = -90;
                break;
            case "j6_Link":
                targetRotation.x = targetValue;
                targetRotation.y = 0;
                targetRotation.z = 90;
                break;
            default:
                return; // Si el nombre de la articulación no coincide, no se aplica ningún cambio.
        }
        // Normaliza los ángulos de la rotación objetivo.
        targetRotation.x = NormalizeAngle(targetRotation.x);
        targetRotation.y = NormalizeAngle(targetRotation.y);
        targetRotation.z = NormalizeAngle(targetRotation.z);
        // Aplica la rotación objetivo directamente al objeto.
        urdfJoint.transform.localEulerAngles = targetRotation;
        // Actualiza el ángulo actual con el valor objetivo.
        currentAngle = targetValue;
    }

    // Obtiene el ángulo actual de la articulación a partir de la transformación.
    // Para ciertas articulaciones se devuelve el ángulo del eje Y, para otras el del eje X.
    private float GetCurrentAngleFromTransform()
    {
        Vector3 rot = urdfJoint.transform.localEulerAngles;
        switch (JointName)
        {
            case "j1_Link":
            case "j3_Link":
            case "j4_Link":
                return rot.y;
            case "j2_Link":
            case "j5_Link":
            case "j6_Link":
                return rot.x;
            default:
                return 0f;
        }
    }

    // Normaliza un ángulo para que se encuentre en el rango [-180, 180].
    private float NormalizeAngle(float angle)
    {
        angle = Mathf.Repeat(angle, 360f);
        if (angle > 180f)
            angle -= 360f;
        return angle;
    }
}
