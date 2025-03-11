/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/
using UnityEngine;
using UnityEngine.UI;

public class ButtonPanel : MonoBehaviour
{
    public GameObject cartesianPanel;
    public GameObject jointPanel;
    public GameObject recordPanel;
    public GameObject txtPanel;
    public GameObject manualModePanel;
    public GameObject autoModePanel;

    public GameObject manualButtonImage;
    public GameObject autoButtonImage;

    public CartesianStateWriterNew cartesianWriter;
    public JointPositionSubscriber jointPositionSubscriber;

    private bool lastManualState;
    private bool lastAutoState;

    void Start()
    {
        manualModePanel.SetActive(true);
        autoModePanel.SetActive(false);
        UpdateButtonVisuals();
    }

    public void OpenCartesianPanel()
    {
        OpenPanel(cartesianPanel);
    }

    public void OpenJointPanel()
    {
        OpenPanel(jointPanel);
    }

    public void OpenRecordPanel()
    {
        OpenPanel(recordPanel);
    }

    public void OpenTXTPanel()
    {
        OpenPanel(txtPanel);
    }

    public void ToggleManualMode()
    {
        bool newState = !manualModePanel.activeSelf;
        manualModePanel.SetActive(newState);
        autoModePanel.SetActive(!newState);
        UpdateButtonVisuals();
    }

    public void ToggleAutoMode()
    {
        bool newState = !autoModePanel.activeSelf;
        autoModePanel.SetActive(newState);
        manualModePanel.SetActive(!newState);
        UpdateButtonVisuals();
    }

    private void OpenPanel(GameObject panelToOpen)
    {
        // Activa sólo el panel seleccionado y desactiva los demás
        cartesianPanel.SetActive(panelToOpen == cartesianPanel);
        jointPanel.SetActive(panelToOpen == jointPanel);
        recordPanel.SetActive(panelToOpen == recordPanel);
        txtPanel.SetActive(panelToOpen == txtPanel);

        // Si es jointPanel, recordPanel o txtPanel, se comportan igual que el panel articular:
        // Se activa la actualización normal de articulaciones y cartesiano.
        if (panelToOpen == jointPanel ||
            panelToOpen == recordPanel ||
            panelToOpen == txtPanel)
        {
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StartUpdating(); // Activa actualización normal en el panel articular.
            if (cartesianWriter != null)
                cartesianWriter.StartUpdating();         // Desactiva interpolación en el panel cartesiano (o lógica que tú uses).
        }
        // Si es el panel cartesiano, se detiene la actualización normal y se activa la interpolación.
        else if (panelToOpen == cartesianPanel)
        {
            if (cartesianWriter != null)
                cartesianWriter.StopUpdating();  // Activa interpolación en el panel cartesiano.
            if (jointPositionSubscriber != null)
                jointPositionSubscriber.StopUpdating(); // Detiene actualización en el panel articular.
            // Además, aquí se reinicia la suscripción de jointPositionSubscriber sólo cuando se presione Send (desde el otro script).
        }
    }

    private void UpdateButtonVisuals()
    {
        if (manualButtonImage != null && autoButtonImage != null)
        {
            manualButtonImage.SetActive(manualModePanel.activeSelf);
            autoButtonImage.SetActive(autoModePanel.activeSelf);
        }
    }
}
