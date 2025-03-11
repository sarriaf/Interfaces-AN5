/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using RosSharp.RosBridgeClient;

public class Control_Cartesiano : MonoBehaviour
{
    public GameObject coordinatesPrefab; // Prefab para mostrar coordenadas
    public RectTransform content; // Contenedor para las coordenadas
    private TMP_InputField[] inputFields = new TMP_InputField[6]; // Vector de casillas de las coordenadas
    private float[] values = new float[6]; // Valores de las coordenadas
    public int numCoordinates; // Número de coordenadas agregadas
    private GameObject[] points = new GameObject[100]; // Array para almacenar los puntos instanciados

    private GameObject robot; // Referencia al robot en la escena

    // Start is called before the first frame update
    void Start()
    {
        // Inicializar las referencias a las casillas de coordenadas
        for (int i = 0; i < 6; i++)
        {
            string coordinateName = NumToCoor(i);
            GameObject foundObject = GameObject.Find(coordinateName);
            if (foundObject != null)
            {
                inputFields[i] = foundObject.GetComponent<TMP_InputField>();
                if (inputFields[i] == null)
                {
                    Debug.LogError("No se encontró un TMP_InputField en " + coordinateName);
                }
            }
            else
            {
                Debug.LogError("No se encontró el GameObject: " + coordinateName);
            }
        }

        numCoordinates = 0;

        // Encontrar el GameObject del robot por su nombre en la jerarquía
        robot = GameObject.Find("fr5v6"); // Asegúrate de que el robot tenga este nombre en la jerarquía
        if (robot == null)
        {
            Debug.LogError("No se encontró el GameObject del robot con el nombre 'fr5v6'.");
        }
    }

    // Update is called once per frame
    void Update()
    {
        // No hay lógica implementada en Update actualmente
    }

    // Método público para agregar una nueva coordenada
    public void AddCoordinate()
    {
        Debug.Log("Iniciando AddCoordinate");

        // Validar y parsear las coordenadas desde las casillas de entrada
        for (int i = 0; i < 6; i++)
        {
            if (!float.TryParse(inputFields[i].text, out values[i]))
            {
                Debug.LogError("Error al convertir la coordenada: " + inputFields[i].name);
                return;
            }
            else
            {
                Debug.Log("Valor de " + inputFields[i].name + ": " + values[i]);
            }
        }

        // Instanciar el prefab de coordenadas en el contenedor
        if (numCoordinates < points.Length)
        {
            points[numCoordinates] = Instantiate(coordinatesPrefab, content, false);
            points[numCoordinates].transform.name = "Coordinate" + numCoordinates;
            points[numCoordinates].transform.localPosition = new Vector3(0, -50 * numCoordinates, 0);

            Debug.Log("Coordenada " + numCoordinates + " creada.");

            // Asignar los valores de las coordenadas al prefab instanciado
            for (int i = 0; i < 6; i++)
            {
                string coordinateName = NumToCoor(i);
                Transform coordTransform = points[numCoordinates].transform.Find("V" + coordinateName);
                if (coordTransform != null)
                {
                    TMP_Text text = coordTransform.GetComponent<TMP_Text>();
                    if (text != null)
                    {
                        text.text = values[i].ToString("F2"); // Formatear a dos decimales
                        Debug.Log("Coordenada " + coordinateName + ": " + values[i]);
                    }
                    else
                    {
                        Debug.LogError("No se encontró el componente TMP_Text en V" + coordinateName + " de " + points[numCoordinates].name);
                    }
                }
                else
                {
                    Debug.LogError("No se encontró el componente V" + coordinateName + " en " + points[numCoordinates].name);
                }
            }

            // Asignar el número de trayectoria
            Transform trajTransform = points[numCoordinates].transform.Find("NumTra");
            if (trajTransform != null)
            {
                TMP_Text trajText = trajTransform.GetComponent<TMP_Text>();
                if (trajText != null)
                {
                    trajText.text = (numCoordinates + 1).ToString();
                }
                else
                {
                    Debug.LogError("No se encontró el componente TMP_Text en NumTra de " + points[numCoordinates].name);
                }
            }
            else
            {
                Debug.LogError("No se encontró el componente NumTra en " + points[numCoordinates].name);
            }

            // Configurar el tamaño del contenedor para acomodar las nuevas coordenadas
            ConfigureContent(70 + (70 * numCoordinates));
            numCoordinates++;

            // Enviar coordenadas al robot
            SendCoordinates(values);
        }
        else
        {
            Debug.LogError("Se alcanzó el límite máximo de coordenadas (100).");
        }
    }

    // Método para convertir un número a su correspondiente nombre de coordenada
    public string NumToCoor(int number)
    {
        switch (number)
        {
            case 0: return "PosX";
            case 1: return "PosY";
            case 2: return "PosZ";
            case 3: return "OriX";
            case 4: return "OriY";
            case 5: return "OriZ";
            default: return "";
        }
    }

    // Método para configurar el tamaño del contenedor de coordenadas
    private void ConfigureContent(float height)
    {
        content.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, height);
    }

    // Método para enviar las coordenadas al robot utilizando cinemática inversa
    private void SendCoordinates(float[] coords)
    {
        Debug.Log("Iniciando SendCoordinates");

        if (robot == null)
        {
            Debug.LogError("Referencia al robot no está asignada.");
            return;
        }

        // Conversión de coordenadas a matriz de transformación
        Matrix4x4 T = Matrix4x4.TRS(
            new Vector3(coords[0], coords[1], coords[2]),
            Quaternion.Euler(coords[3], coords[4], coords[5]),
            Vector3.one
        );

        // Cálculo de los ángulos de las articulaciones usando el modelo geométrico inverso
        float[] q = RobotKinematics.MgiAn5(T);

        // Enviar los ángulos calculados a las articulaciones del robot
        for (int i = 0; i < q.Length; i++)
        {
            string jointName = "j" + (i + 1) + "_Link";
            GameObject joint = GameObject.Find(jointName);
            if (joint != null)
            {
                JointStateWriter writer = joint.GetComponent<JointStateWriter>();
                if (writer != null)
                {
                    writer.Write(q[i]);
                }
                else
                {
                    Debug.LogError("No se encontró el componente JointStateWriter en " + jointName);
                }
            }
            else
            {
                Debug.LogError("No se encontró el GameObject de la articulación: " + jointName);
            }
        }

        Debug.Log("Coordenadas enviadas al robot.");
    }
}
