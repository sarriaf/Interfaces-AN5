import rclpy
from rclpy.node import Node
from frhal_msgs.srv import ROSCmdInterface
import csv
import time
from std_msgs.msg import String

class CartesianPointSender(Node):
    def __init__(self):
        super().__init__('cartesian_point_sender')
        self.client = self.create_client(ROSCmdInterface, '/FR_ROS_API_service')
        self.stop_motion = False  # Bandera para detener la ejecución

        # Espera a que el servicio esté disponible.
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /FR_ROS_API_service...')
        self.get_logger().info('Servicio conectado.')

        # Variable para almacenar la posición articular actual.
        self.current_joint_positions = None

        # Crear suscripción al tópico que publica la posición articular actual.
        self.subscription = self.create_subscription(
            String,
            '/current_joint_position',
            self.current_joint_callback,
            10
        )
        
        # Suscripción al tópico api_command para detener la ejecución si se recibe StopMotion()
        self.api_command_subscriber = self.create_subscription(
            String,
            '/api_command',
            self.api_command_callback,
            10
        )

        # Inicia el envío de comandos.
        self.send_cartesian_points()

    def current_joint_callback(self, msg):
        """Callback para actualizar la posición articular actual.
        
        Se espera que el mensaje tenga el formato:
            data: "-22.4,-52.78,95.0,-222.22,22.4,135.0"
        """
        try:
            joints = [float(x.strip()) for x in msg.data.split(',')]
            self.current_joint_positions = joints
            self.get_logger().debug(f'Posiciones articulares actuales: {joints}')
        except Exception as e:
            self.get_logger().error(f'Error al parsear las posiciones articulares: {e}')

    def api_command_callback(self, msg):
        """Callback que escucha el tópico api_command y, si se detecta el comando StopMotion(), activa la bandera de detención."""
        if msg.data.strip() == "StopMotion()":
            self.get_logger().info("Comando StopMotion recibido. Deteniendo ejecución del script...")
            self.stop_motion = True

    def read_cartesian_points(self, file_path):
        """Lee el archivo de posiciones cartesianas y extrae 8 columnas:
        x, y, z, rx, ry, rz, speed y wait (tiempo de espera)."""
        points = []
        try:
            with open(file_path, newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                for row in reader:
                    if len(row) < 8:
                        continue  # Se requiere al menos 8 columnas
                    points.append({
                        'x': float(row[0]),
                        'y': float(row[1]),
                        'z': float(row[2]),
                        'rx': float(row[3]),
                        'ry': float(row[4]),
                        'rz': float(row[5]),
                        'speed': float(row[6]),  # Velocidad para MoveL
                        'wait': float(row[7])    # Tiempo de espera (en segundos)
                    })
        except Exception as e:
            self.get_logger().error(f'Error al leer el archivo de puntos cartesianos: {e}')
        return points

    def read_joint_positions(self, file_path):
        """Lee el archivo de posiciones articulares.
        
        Se espera que cada línea contenga al menos 6 valores (para 6 joints)
        separados por comas.
        """
        joint_positions = []
        try:
            with open(file_path, newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                for row in reader:
                    if len(row) < 6:
                        continue  # Se requiere al menos 6 columnas
                    joints = [float(val) for val in row[:6]]
                    joint_positions.append(joints)
        except Exception as e:
            self.get_logger().error(f'Error al leer el archivo de posiciones articulares: {e}')
        return joint_positions

    def joints_close(self, current, desired, tol=1.0):
        """Compara dos listas de posiciones articulares y devuelve True si la
        diferencia absoluta de cada joint es menor que 'tol' (por defecto 1.0)."""
        if len(current) != len(desired):
            return False
        return all(abs(c - d) < tol for c, d in zip(current, desired))

    def wait_until_reached(self, desired_joint, tolerance=1.0, timeout=60):
        """
        Espera (hasta 'timeout' segundos) hasta que la posición articular actual
        esté dentro de 'tolerance' unidades de la posición deseada.
        """
        self.get_logger().info(f'Esperando a alcanzar la posición articular deseada: {desired_joint}')
        start_time = time.time()
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            # Verificar si se recibió el comando StopMotion
            if self.stop_motion:
                self.get_logger().info("Ejecución detenida por comando StopMotion durante la espera.")
                break
            if self.current_joint_positions is not None:
                if self.joints_close(self.current_joint_positions, desired_joint, tolerance):
                    self.get_logger().info("Posición articular alcanzada.")
                    break
            if time.time() - start_time > timeout:
                self.get_logger().error("Tiempo de espera excedido sin alcanzar la posición deseada.")
                break
            time.sleep(0.1)

    def send_command(self, command):
        """Envía un comando al servicio y espera la respuesta."""
        req = ROSCmdInterface.Request()
        req.cmd_str = command

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Respuesta del servicio: {response.cmd_res}')
        else:
            self.get_logger().error('Error al llamar al servicio.')

    def send_cartesian_points(self):
        # Rutas de los archivos (ajústalas según tu entorno)
        cartesian_file = '/home/tarw/Interfaz AppDesigner AN5/python_position.txt'
        joint_file = '/home/tarw/Interfaz AppDesigner AN5/joint_python_position.txt'
        
        points = self.read_cartesian_points(cartesian_file)
        joint_positions = self.read_joint_positions(joint_file)

        if not points:
            self.get_logger().error('No se encontraron puntos válidos en el archivo de posiciones cartesianas.')
            return
        if len(points) != len(joint_positions):
            self.get_logger().error('El número de puntos cartesianos y posiciones articulares no coincide.')
            return

        # Separamos en lotes (batch) de 5 puntos si fuera necesario
        batch_size = 5
        batches = [points[i:i + batch_size] for i in range(0, len(points), batch_size)]
        joint_batches = [joint_positions[i:i + batch_size] for i in range(0, len(joint_positions), batch_size)]

        for batch_idx, (batch_points, batch_joints) in enumerate(zip(batches, joint_batches), start=1):
            # Verificar si se ha recibido el comando StopMotion antes de iniciar el lote
            if self.stop_motion:
                self.get_logger().info("Ejecución detenida por comando StopMotion. Abortando lotes...")
                break

            self.get_logger().info(f'Procesando lote {batch_idx}...')

            # 1. Enviar comandos CARTPoint para almacenar cada punto en el robot.
            for i, point in enumerate(batch_points, start=1):
                if self.stop_motion:
                    self.get_logger().info("Ejecución detenida por comando StopMotion. Abortando envío de puntos...")
                    break
                cmd = f"CARTPoint({i},{point['x']},{point['y']},{point['z']},{point['rx']},{point['ry']},{point['rz']})"
                self.send_command(cmd)
                self.get_logger().info(f'Punto almacenado en lote {batch_idx}: {cmd}')

            # 2. Enviar comandos MoveL secuencialmente.
            for i, (point, desired_joint) in enumerate(zip(batch_points, batch_joints), start=1):
                if self.stop_motion:
                    self.get_logger().info("Ejecución detenida por comando StopMotion. Abortando comandos MoveL...")
                    break
                move_cmd = f"MoveL(CART{i},{point['speed']})"
                self.send_command(move_cmd)
                self.get_logger().info(f'Comando enviado en lote {batch_idx}: {move_cmd}')
                
                # Espera hasta que el robot alcance la posición articular deseada.
                self.wait_until_reached(desired_joint)
                if self.stop_motion:
                    break
                
                # Espera adicional según lo indicado en la octava columna.
                wait_time = point.get('wait', 0)
                self.get_logger().info(f'Esperando {wait_time} segundos antes de enviar el siguiente comando...')
                time.sleep(wait_time)

            if self.stop_motion:
                break

        if self.stop_motion:
            self.get_logger().info('Trayectoria interrumpida. Cerrando nodo...')
        else:
            self.get_logger().info('Trayectoria completada. Cerrando nodo...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPointSender()
    # No es necesario llamar a rclpy.spin(node) ya que el nodo se cierra al finalizar la trayectoria.

if __name__ == '__main__':
    main()