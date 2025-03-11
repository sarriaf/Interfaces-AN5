/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/
using UnityEngine;

public class Observador : MonoBehaviour
{
    // Referencias a las 4 cámaras
    public Camera camara1;
    public Camera camara2;
    public Camera camara3;


    public float VelMov;
    public float VelRot;

    Vector3 Movimiento;
    Vector2 Rotacion;
    Vector3 PosicionInicial;
    Quaternion RotacionInicial;
    
void Start()
{
    // Activa solo la cámara 1 al inicio
    MostrarSoloCamara(camara1);
}

    void Awake()
    {
        PosicionInicial = transform.position;
        RotacionInicial = transform.rotation;
    }

    void Update()
    {
        // Movimiento
        Movimiento.x = Input.GetAxis("Horizontal")* VelMov * Time.deltaTime;
        Movimiento.y = Input.GetAxis("Vertical")* VelMov * Time.deltaTime;
        Movimiento.z = Input.GetAxis("Altura")* VelMov * Time.deltaTime;
        Rotacion.x = Input.GetAxis("RotHorizontal")* VelRot * Time.deltaTime;

        MoverVista();

        // Reiniciar vista al pulsar R
        if(Input.GetKeyDown(KeyCode.R))
        {
            ReiniciarVista();
        }

        // Conmutar cámaras con las teclas 1, 2, 3 y 4
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            MostrarSoloCamara(camara1);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            MostrarSoloCamara(camara2);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            MostrarSoloCamara(camara3);
        }
        
    }

    void MoverVista()
    {
        transform.Translate(Movimiento.x, Movimiento.z, Movimiento.y);
        transform.Rotate(0, Rotacion.x, 0);
    }

    void ReiniciarVista()
    {
        transform.position = PosicionInicial;
        transform.rotation = RotacionInicial;
    }

    // Función para mostrar sólo la cámara indicada y apagar las demás
    void MostrarSoloCamara(Camera camaraActiva)
    {
        // Apagar todas
        camara1.enabled = false;
        camara2.enabled = false;
        camara3.enabled = false;

        // Encender la que se quiere mostrar
        camaraActiva.enabled = true;
    }
}
