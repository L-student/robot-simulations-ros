import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from time import sleep
import numpy as np 
from scipy.spatial.transform import Rotation


def quaternion_rotation_matrix(T):
    Q = T

    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
    
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    
    return rot_matrix


def forward_kinematics_dh(joint_positions):
    dh_params = np.array([
        [joint_positions[0], 0.125, 0, np.pi/2],
        [joint_positions[1] + np.pi/2, 0, 0.21, 0],
        [joint_positions[2] - np.pi/2, 0, -0.075, -np.pi/2],
        [joint_positions[3], 0.21, 0, np.pi/2],
        [joint_positions[4], 0, 0, -np.pi/2],
        [joint_positions[5], 0.07, 0, 0]
    ])
    T = np.eye(4)
    for i in range(len(joint_positions)):
        theta, d,  a, alpha,  = dh_params[i]
        A_i = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        # Multiplicar a matriz de transformação homogênea pela matriz acumulada
        T = A_i @ T

    return T


class DensoControl(Node):
    def __init__(self):
        super().__init__('denso_control')
        
        # Defining class variables
        self.pose = Pose()
        self.joint_states = JointState()
        
        # Creating subscribers and relating respective feedback functions
        self.sub_joints = self.create_subscription(JointState, '/denso/joint_states', self.fb_joints, 10)
        self.sub_pose = self.create_subscription(Pose, '/denso/pose', self.fb_pose, 10)

        # Creating publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, '/denso/target_positions', 10)


    def fb_joints(self, msg):
        self.joint_states = msg
        self.get_logger().debug('Received : "%s"' % msg.position)

    def fb_pose(self, msg):
        self.pose = msg
        self.get_logger().debug('Received : "%s"' % msg.position)
    
    def publish_setpoint(self, q):
        # Criando mensagem
        msg = Float64MultiArray()
        msg.data = q
        
        # Mandando sinal de atuação no robo
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    denso_control = DensoControl()

    # Enviando comando de posição
    #q_d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #q_d = np.array([np.pi/3, np.pi/3, -np.pi/10, -np.pi/6, np.pi/6, np.pi/3])
    #q_d = np.array([-np.pi/3, np.pi/4, 3*np.pi/8, -np.pi/6, -3*np.pi/17, np.pi/10])
    #q_d = np.array([-3*np.pi/4, -np.pi/4, np.pi/8, 7*np.pi/19, 0, 0])
    q_d = np.array([0, 0, -np.pi/9, 0, 0, 0])
    denso_control.publish_setpoint(q_d.tolist())

    # Recebendo denso joints e pose e esperando o manipulador chegar na posição desejada
    rclpy.spin_once(denso_control)

    q_c = np.array(denso_control.joint_states.position)
    while (np.linalg.norm(q_c - q_d) > 1): # Esperando com uma tolerancia de 1 no erro de regime permanente
        sleep(1)
        rclpy.spin_once(denso_control)
        q_c = np.array(denso_control.joint_states.position)
    denso_control.get_logger().info("Robot is ready")

    print(q_c)

    T = forward_kinematics_dh(q_d)
    print("Matriz de Transformação Homogênea obtida:\n")
    print(T)

    T2 = T[:3, :3]
    matriz_posicao = np.array([denso_control.pose.position.x, denso_control.pose.position.y, denso_control.pose.position.z])
    matriz_calculada = np.dot(T2, np.array([0, 0, 0]))
    diferenca = np.linalg.norm(matriz_posicao - matriz_calculada)
    denso_control.get_logger().info(f"Pose, diferença:\n {diferenca}")

    
    quaternion_sistema = [
        denso_control.pose.orientation.w,
        denso_control.pose.orientation.x,
        denso_control.pose.orientation.y,
        denso_control.pose.orientation.z
    ]
    rot = Rotation.from_matrix(T2)
    quaternion_calculado = rot.as_quat()
    denso_control.get_logger().info(f"Quaternion Sistema: \n{quaternion_sistema}")
    denso_control.get_logger().info(f"Quaternion Calculado: \n{quaternion_calculado}")




    denso_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
