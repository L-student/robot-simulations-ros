import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from time import sleep
import numpy as np 

class DensoControl(Node):
    def __init__(self):
        super().__init__('denso_control')
        
        # Defining class variables
        self.pose = Pose()
        self.joint_states = JointState()
        
        # Creating subscribers and relating respective feedback functions
        self.sub_joints = self.create_subscription(JointState, 'denso/joint_states', self.fb_joints, 10)
        self.sub_pose = self.create_subscription(Pose, 'denso/pose', self.fb_pose, 10)

        # Creating publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'denso/target_positions', 10)


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
    q_d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #q_d = np.array([np.pi/3, np.pi/3, -np.pi/10, -np.pi/6, np.pi/6, np.pi/3])
    #q_d = np.array([-np.pi/3, np.pi/4, 3*np.pi/8, -np.pi/6, -3*np.pi/17, np.pi/10])
    #q_d = np.array([-3*np.pi/4, -np.pi/4, np.pi/8, 7*np.pi/19, 0, 0])
    #q_d = np.array([0, 0, -np.pi/9, 0, 0, 0])
    denso_control.publish_setpoint(q_d.tolist())

    # Recebendo denso joints e pose e esperando o manipulador chegar na posição desejada
    rclpy.spin_once(denso_control)

    q_c = np.array(denso_control.joint_states.position)
    while (np.linalg.norm(q_c - q_d) > 1): # Esperando com uma tolerancia de 1 no erro de regime permanente
        sleep(1)
        rclpy.spin_once(denso_control)
        q_c = np.array(denso_control.joint_states.position)
    denso_control.get_logger().info("Robot is ready")

    ### Insira o código aqui
    #
    #
    #
    #
    ###
        
    denso_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
