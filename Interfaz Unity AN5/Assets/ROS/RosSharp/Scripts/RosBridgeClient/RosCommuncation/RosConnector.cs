/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing,
software distributed under the License is distributed on an "AS IS",
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Threading;
using RosSharp.RosBridgeClient.Protocols;
using UnityEngine;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        [Header("Connection Settings")]
        public string RosBridgeServerUrl = "ws://localhost:9090";
        public Protocol protocol;
        public RosSocket.SerializerEnum Serializer;

        [Header("Timeouts")]
        public int SecondsTimeout = 20;
        public int ReconnectDelaySeconds = 5;

        [Header("UI Components (Opcional)")]
        public Text statusText;

        [Header("UI Button (Opcional)")]
       
        public Button reconnectButton;

        public RosSocket RosSocket { get; private set; }
        public ManualResetEvent IsConnected { get; private set; }

        public bool IsOnline
        {
            get { return IsConnected != null && IsConnected.WaitOne(0); }
        }

        private bool isReconnecting = false;
        private Thread connectionThread;

        public virtual void Awake()
        {
            IsConnected = new ManualResetEvent(false);

            // Iniciamos la conexión en un hilo separado (auto reconexión con bucle y retraso)
            connectionThread = new Thread(ConnectAndWait);
            connectionThread.Start();
        }

        private void Start()
        {
        
            if (reconnectButton != null)
                reconnectButton.onClick.AddListener(ReconnectNow);
        }

        private void Update()
        {
          
            if (statusText != null)
            {
                if (IsOnline)
                {
                    statusText.text = "ONLINE";
                    statusText.color = Color.green;
                }
                else
                {
                    statusText.text = "OFFLINE";
                    statusText.color = Color.red;
                }
            }
        }

        // Bucle para auto-reconexión con un retardo (se inicia en Awake y en OnClosed)
        private void ConnectAndWait()
        {
            while (true)
            {
                RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);

                if (!IsConnected.WaitOne(SecondsTimeout * 1000))
                {
                    Debug.LogWarning("Failed to connect to RosBridge at: " + RosBridgeServerUrl);
                }
                else
                {
                    // Conexión exitosa, salir del bucle
                    break;
                }

                Debug.Log("Retrying connection in " + ReconnectDelaySeconds + " seconds...");
                Thread.Sleep(ReconnectDelaySeconds * 1000);
            }
        }

        // Método estático para crear el RosSocket y enlazar OnConnected y OnClosed
        public static RosSocket ConnectToRos(
            Protocol protocolType,
            string serverUrl,
            EventHandler onConnected = null,
            EventHandler onClosed = null,
            RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.Microsoft)
        {
            IProtocol protocol = ProtocolInitializer.GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;
            return new RosSocket(protocol, serializer);
        }

        // Método para reconexión inmediata, sin esperar ReconnectDelaySeconds
        public void ReconnectNow()
        {
            if (IsOnline)
            {
                Debug.Log("Already connected. No need to reconnect.");
                return;
            }

            Debug.Log("Manual reconnect now...");

            // Cancelamos el hilo que pudiera estar en medio del bucle de auto reconexión
            if (connectionThread != null && connectionThread.IsAlive)
            {
                Debug.Log("Aborting the auto-reconnect thread...");
                connectionThread.Abort();
            }

            // Cerramos el socket anterior si seguía abierto
            if (RosSocket != null)
                RosSocket.Close();

            // Reset para poder volver a esperar la señal
            IsConnected.Reset();
            isReconnecting = false;

            // Iniciamos un hilo nuevo que haga un único intento de conexión
            connectionThread = new Thread(ConnectOnce);
            connectionThread.Start();
        }

        // Únicamente intenta conectar una vez, sin bucles de reintento
        private void ConnectOnce()
        {
            RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);

            // Esperamos a que se establezca la conexión (o agotar el timeout)
            if (!IsConnected.WaitOne(SecondsTimeout * 1000))
            {
                Debug.LogWarning("Failed to connect (manual attempt) to RosBridge at: " + RosBridgeServerUrl);
            }
            else
            {
                Debug.Log("Connected to RosBridge (manual attempt): " + RosBridgeServerUrl);
            }
        }

        // Evento que se llama cuando se cierra la conexión
        private void OnClosed(object sender, EventArgs e)
        {
            IsConnected.Reset();
            Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl);

            // Lógica de auto reconexión (con retardo)
            if (!isReconnecting)
            {
                isReconnecting = true;
                connectionThread = new Thread(ConnectAndWait);
                connectionThread.Start();
            }
        }

        // Evento que se llama cuando la conexión se realiza con éxito
        private void OnConnected(object sender, EventArgs e)
        {
            IsConnected.Set();
            isReconnecting = false;
            Debug.Log("Connected to RosBridge: " + RosBridgeServerUrl);
        }

        private void OnApplicationQuit()
        {
            if (RosSocket != null)
                RosSocket.Close();

            if (connectionThread != null && connectionThread.IsAlive)
                connectionThread.Abort();
        }
    }
}
