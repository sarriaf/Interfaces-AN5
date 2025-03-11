/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/

using UnityEngine;

public class RobotKinematics : MonoBehaviour
{
    // Método estático para calcular la matriz de transformación total usando los parámetros DH y los ángulos articulares q
    public static Matrix4x4 MgdAn5(float[] q)
    {
        // Dimensiones DH del robot AN5 (Denavit-Hartenberg)
        float[,] DH_params = new float[,] {
            {0f, Mathf.PI / 2f, 0.152f, q[0]},
            {-0.425f, 0f, 0f, q[1]},
            {-0.395f, 0f, 0f, q[2]},
            {0f, Mathf.PI / 2f, 0.102f, q[3]},
            {0f, -Mathf.PI / 2f, 0.102f, q[4]},
            {0f, 0f, 0.267f, q[5]}
        };

        Matrix4x4 T = Matrix4x4.identity; // Inicializar la matriz de transformación total como identidad

        // Calcular la matriz de transformación para cada articulación y acumularla en T
        for (int i = 0; i < DH_params.GetLength(0); i++)
        {
            float theta = DH_params[i, 3];
            float d = DH_params[i, 2];
            float a = DH_params[i, 0];
            float alpha = DH_params[i, 1];

            Matrix4x4 A = DhTransform(a, alpha, d, theta); // Obtener la matriz DH para la articulación actual

            T *= A; // Multiplicar la matriz total por la matriz de la articulación actual
        }

        return T; // Devolver la matriz de transformación total
    }

    // Método estático para calcular los ángulos articulares q a partir de la matriz de transformación total T
    public static float[] MgiAn5(Matrix4x4 T)
    {
        // Parámetros DH del robot AN5
        float a2 = -0.244f;
        float a3 = -0.213f;
        float d1 = 0.152f;
        float d4 = 0.112f;
        //float d5 = 0.085f;
        //float d6 = 0.082f;

        // Inicialización de variables para los ángulos articulares
        float[] q = new float[6];

        // Extracción de la posición del efector final desde la matriz T
        float px = T.m03;
        float py = T.m13;
        float pz = T.m23;

        // Cálculo del ángulo q1 usando atan2 para obtener el ángulo correcto en el primer cuadrante
        float q1 = Mathf.Atan2(py, px);

        // Cálculo de la posición en el plano de trabajo
        float r1 = Mathf.Sqrt(px * px + py * py);
        float r2 = pz - d1;

        // Cálculo del ángulo q3 utilizando la ley de cosenos
        float D = (r1 * r1 + r2 * r2 - a2 * a2 - a3 * a3) / (2f * a2 * a3);
        float q3 = Mathf.Atan2(Mathf.Sqrt(1f - D * D), D);

        // Cálculo del ángulo q2 utilizando la ley de cosenos y los valores calculados anteriormente
        float q2 = Mathf.Atan2(r2, r1) - Mathf.Atan2(a3 * Mathf.Sin(q3), a2 + a3 * Mathf.Cos(q3));

        // Cálculo de la matriz de rotación R03 hasta el tercer eslabón
        Matrix4x4 R03 = DhTransform(a2, 0f, d1, q1) *
                        DhTransform(a3, 0f, 0f, q2) *
                        DhTransform(0f, Mathf.PI / 2f, d4, q3);

        // Cálculo de la matriz de rotación R36 desde el tercer eslabón hasta el efector final
        Matrix4x4 R36 = R03.inverse * T;

        // Cálculo de los ángulos q4, q5 y q6 usando la matriz de rotación R36
        float q4 = Mathf.Atan2(R36.m12, R36.m02);
        float q5 = Mathf.Atan2(Mathf.Sqrt(R36.m02 * R36.m02 + R36.m12 * R36.m12), R36.m22);
        float q6 = Mathf.Atan2(R36.m21, -R36.m20);

        // Asignar los ángulos calculados al arreglo q
        q[0] = q1;
        q[1] = q2;
        q[2] = q3;
        q[3] = q4;
        q[4] = q5;
        q[5] = q6;

        return q; // Devolver el arreglo de ángulos articulares
    }

    // Método auxiliar para calcular la matriz de transformación de Denavit-Hartenberg
    private static Matrix4x4 DhTransform(float a, float alpha, float d, float theta)
    {
        Matrix4x4 A = new Matrix4x4(
            new Vector4(Mathf.Cos(theta), -Mathf.Sin(theta) * Mathf.Cos(alpha), Mathf.Sin(theta) * Mathf.Sin(alpha), a * Mathf.Cos(theta)),
            new Vector4(Mathf.Sin(theta), Mathf.Cos(theta) * Mathf.Cos(alpha), -Mathf.Cos(theta) * Mathf.Sin(alpha), a * Mathf.Sin(theta)),
            new Vector4(0f, Mathf.Sin(alpha), Mathf.Cos(alpha), d),
            new Vector4(0f, 0f, 0f, 1f)
        );

        return A; // Devolver la matriz de transformación DH
    }
}
