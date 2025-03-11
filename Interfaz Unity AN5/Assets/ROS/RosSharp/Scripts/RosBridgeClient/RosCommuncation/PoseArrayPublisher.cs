using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections;
using System.Collections.Generic;
// TM 
using TMPro;

namespace RosSharp.RosBridgeClient
{
    public class PoseArrayPublisher : UnityPublisher<MessageTypes.Geometry.Pose>
    {

        //private Vector3 Pos;                                            // Vector de posiciones
        //private Quaternion Ori;                                         // Vector de orientaciones
        public TMP_InputField[] coordenadas = new TMP_InputField[6];    // Vector de casillas
        private float[] var = new float[6];                             // Var
        //private List<float> var2 = new List<float>();
        private int num;       
        //private Pose[] posi;                                         // Iterador
        private List<MessageTypes.Geometry.Pose> poses2 = new List<MessageTypes.Geometry.Pose>();

        public string FrameId = "Unity";

        private MessageTypes.Geometry.Pose posicion;
        //private MessageTypes.Geometry.PoseArray message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
            num = 0;
        }

        /*private void FixedUpdate()
        {
            UpdateMessage();
        }*/

        private void InitializeMessage()
        {
            posicion = new MessageTypes.Geometry.Pose();

            /*message = new MessageTypes.Geometry.PoseArray
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = FrameId
                }
            };*/
            //message.poses = new MessageTypes.Geometry.Pose[5];
        }

        public void coordinates()
        {
            for(int i = 0; i < 6; i++)
            {
                var[i] = float.Parse(coordenadas[i].text);
                //var2.Add(float.Parse(coordenadas[i].text));
                //Debug.Log(float.Parse(var[i]));
            }
            CargarCoordenada();
        }

        public void UpdateMessage()
        {
            //message.header.Update();
            //Debug.Log(message.poses);
            foreach (MessageTypes.Geometry.Pose item in poses2)
            {
                Publish(item);
            }
            
        }

        private static void GetGeometryPoint(float[] position, MessageTypes.Geometry.Point geometryPoint)
        {
            geometryPoint.x = position[0];
            geometryPoint.y = position[1];
            geometryPoint.z = position[2];
        }

        private static void GetGeometryQuaternion(float[] quaternion, MessageTypes.Geometry.Quaternion geometryQuaternion)
        {
            geometryQuaternion.x = quaternion[3];
            geometryQuaternion.y = quaternion[4];
            geometryQuaternion.z = quaternion[5];
            geometryQuaternion.w = 0;
        }

        public void CargarCoordenada()
        {
            GetGeometryPoint(var, posicion.position);
            GetGeometryQuaternion(var, posicion.orientation);
            Debug.Log(num);
            //message.poses[num] = posicion;
            poses2.Add(posicion);
            //poses2.Insert(num,posicion);
        }

        public void BorrarCoordenada()
        {

        }
        
        public void AgregarCoordenada()
        {
            num = num+1;
        }

        public void EliminarCoordenada()
        {
            num = num-1;
        }

    }
}
