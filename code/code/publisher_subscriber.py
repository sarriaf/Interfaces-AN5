import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from frhal_msgs.srv import ROSCmdInterface
import xmlrpc.client
import socket
import threading
import time


# Clase que representa el Robot FRC
class FRCRobot:
    # Constructor de la clase
    def __init__(self, ip):
        # Dirección del servidor XML-RPC
        self.server_url = f"http://{ip}:20003/RPC2"
        # Cliente XML-RPC para comunicarse con el robot
        self.client = xmlrpc.client.ServerProxy(self.server_url, allow_none=True)

    # Método para obtener las posiciones de las articulaciones
    def get_joint_positions(self):
        try:
            # Solicita las posiciones articulares al servidor
            joint_pos = self.client.GetActualJointPosDegree(1)
            # Redondea las posiciones a dos decimales
            joint_positions = [round(pos, 2) for pos in joint_pos[1:]]
            return joint_positions
        except Exception as e:
            # Muestra un mensaje de error en caso de excepción
            print(f"Error obteniendo posiciones de las articulaciones: {e}")
            return None

    # Método para obtener la posición cartesiana
    def get_cartesian_position(self):
        try:
            # Solicita la posición cartesiana al servidor
            cart_pos = self.client.GetActualTCPPose(1)
            # Redondea la posición a dos decimales y la retorna en un diccionario
            cartesian_position = {
                'x': round(cart_pos[1], 2),
                'y': round(cart_pos[2], 2),
                'z': round(cart_pos[3], 2),
                'rx': round(cart_pos[4], 2),
                'ry': round(cart_pos[5], 2),
                'rz': round(cart_pos[6], 2)
            }
            return cartesian_position
        except Exception as e:
            # Muestra un mensaje de error en caso de excepción
            print(f"Error obteniendo posición cartesiana: {e}")
            return None

# Clase que publica la información del Robot a través de ROS2
class RobotPublisher(Node):
    # Constructor de la clase
    def __init__(self):
        # Inicializa la clase Node de ROS2 con el nombre 'robot_publisher'
        super().__init__('robot_publisher')
        # Crea un publicador para las posiciones articulares (en el tópico 'current_joint_position')
        self.joint_publisher_ = self.create_publisher(String, 'current_joint_position', 10)
        # Crea un publicador para la posición cartesiana (en el tópico 'current_cartesian_position')
        self.cartesian_publisher_ = self.create_publisher(String, 'current_cartesian_position', 10)
        # Instancia de la clase FRCRobot con la IP del robot
        self.robot = FRCRobot("192.168.58.2")
        # Crea un servidor TCP para comunicarse con el robot
        self.tcp_server = TCPServer(self.robot)
        # Crea e inicia el hilo del servidor TCP
        self.tcp_server_thread = threading.Thread(target=self.tcp_server.start_server)
        self.tcp_server_thread.start()
        # Intervalo de tiempo para ejecutar la función timer_callback
        timer_period = 0.15  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Crea un cliente para el servicio de comandos ROSCmdInterface
        self.client = self.create_client(ROSCmdInterface, '/FR_ROS_API_service')
        # Espera hasta que el servicio esté disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando nuevamente...')
        # Crea una suscripción para el tópico 'api_command'
        self.subscription = self.create_subscription(
            String,
            'api_command',
            self.listener_callback,
            10)
        self.subscription  # Evita la advertencia de variable no utilizada

    # Función que se llama periódicamente para publicar los datos del robot
    def timer_callback(self):
        try:
            # Obtiene las posiciones articulares
            joint_positions = self.robot.get_joint_positions()
            if joint_positions:
                # Crea el mensaje con las posiciones articulares y lo publica
                msg = String()
                msg.data = ','.join(map(str, joint_positions))
                self.joint_publisher_.publish(msg)
                # Muestra un mensaje de información (en español)
                self.get_logger().info('Publicando posiciones de las articulaciones: "%s"' % msg.data)

            # Obtiene la posición cartesiana
            cartesian_position = self.robot.get_cartesian_position()
            if cartesian_position:
                # Crea el mensaje con la posición cartesiana y lo publica
                msg = String()
                msg.data = ','.join(map(str, [
                    cartesian_position['x'],
                    cartesian_position['y'],
                    cartesian_position['z'],
                    cartesian_position['rx'],
                    cartesian_position['ry'],
                    cartesian_position['rz']
                ]))
                self.cartesian_publisher_.publish(msg)
                # Muestra un mensaje de información (en español)
                self.get_logger().info('Publicando posición cartesiana: "%s"' % msg.data)
        except Exception as e:
            # Muestra un mensaje de error en caso de excepción (en español)
            self.get_logger().error(f"Error en timer_callback: {e}")

    # Función que se ejecuta cuando se recibe un mensaje en el tópico 'api_command'
    def listener_callback(self, msg):
        command = msg.data
        # Muestra el comando recibido (en español)
        self.get_logger().info(f'Comando recibido: "{command}"')
        self.send_request(command)

    # Envía la solicitud al servicio ROSCmdInterface
    def send_request(self, command):
        # Crea la solicitud para el servicio
        req = ROSCmdInterface.Request()
        req.cmd_str = command
        # Llama al servicio de manera asíncrona
        self.future = self.client.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    # Función que maneja la respuesta del servicio
    def service_response_callback(self, future):
        try:
            # Obtiene la respuesta del servicio
            response = future.result()
            # Muestra la respuesta (en español)
            self.get_logger().info(f'Respuesta del servicio: {response.cmd_res}')
        except Exception as e:
            # Muestra un mensaje de error en caso de excepción (en español)
            self.get_logger().error(f'Falla en la llamada al servicio: {e}')

# Clase que implementa un servidor TCP para enviar datos del robot
class TCPServer:
    # Constructor de la clase
    def __init__(self, robot, host='0.0.0.0', port=20003):
        # Guarda la instancia del robot y los valores de host y puerto
        self.robot = robot
        self.host = host
        self.port = port

    # Inicia el servidor TCP
    def start_server(self):
        # Crea un socket de tipo TCP
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Permite reutilizar la dirección
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Asocia el socket con el host y el puerto
        server_socket.bind((self.host, self.port))
        # Escucha conexiones entrantes, con un backlog de 1
        server_socket.listen(1)
        # Imprime un mensaje indicando que el servidor está escuchando (en español)
        print(f"Servidor TCP escuchando en {self.host}:{self.port}")

        # Bucle principal del servidor
        while True:
            # Acepta una conexión entrante
            client_socket, addr = server_socket.accept()
            # Imprime un mensaje indicando la conexión aceptada (en español)
            print(f"Conexión aceptada de {addr}")
            # Crea e inicia un nuevo hilo para manejar la conexión con el cliente
            threading.Thread(target=self.handle_client, args=(client_socket,)).start()

    # Maneja la comunicación con el cliente conectado
    def handle_client(self, client_socket):
        with client_socket:
            while True:
                try:
                    # Obtiene las posiciones articulares
                    joint_positions = self.robot.get_joint_positions()
                    # Obtiene la posición cartesiana
                    cartesian_position = self.robot.get_cartesian_position()

                    # Envía las posiciones articulares si están disponibles
                    if joint_positions:
                        joint_data = ','.join(map(str, joint_positions))
                        client_socket.sendall(f"Joint Positions: {joint_data}\n".encode('utf-8'))

                    # Envía la posición cartesiana si está disponible
                    if cartesian_position:
                        cart_data = ','.join(map(str, [
                            cartesian_position['x'],
                            cartesian_position['y'],
                            cartesian_position['z'],
                            cartesian_position['rx'],
                            cartesian_position['ry'],
                            cartesian_position['rz']
                        ]))
                        client_socket.sendall(f"Cartesian Position: {cart_data}\n".encode('utf-8'))

                    # Espera un breve periodo antes de enviar nuevamente
                    time.sleep(0.15)

                except Exception as e:
                    # Imprime un mensaje de error en caso de excepción (en español)
                    print(f"Error manejando cliente TCP: {e}")
                    break

# Función principal que inicia el nodo de ROS2
def main(args=None):
    # Inicializa el sistema ROS2
    rclpy.init(args=args)
    # Crea una instancia de RobotPublisher
    robot_publisher = RobotPublisher()
    # Mantiene el nodo en ejecución
    rclpy.spin(robot_publisher)
    # Destruye el nodo antes de cerrar
    robot_publisher.destroy_node()
    # Apaga el sistema ROS2
    rclpy.shutdown()

# Punto de entrada del script
if __name__ == '__main__':
    main()
