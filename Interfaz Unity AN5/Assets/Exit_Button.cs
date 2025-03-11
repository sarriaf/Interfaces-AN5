using UnityEngine;

public class ExitButton : MonoBehaviour
{
    public void ExitGame()
    {
        #if UNITY_EDITOR
            // Aplicable solo en el editor de Unity
            UnityEditor.EditorApplication.isPlaying = false;
        #else
            // Cierra la aplicación
            Application.Quit();
        #endif
    }
}
