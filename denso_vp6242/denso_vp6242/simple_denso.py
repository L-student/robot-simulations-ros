import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

SIMULATION = True
Ts = 0.05

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
        self.timer = self.create_timer(Ts, self.control_loop)


    def fb_joints(self, msg):
        self.joint_states = msg

    def fb_pose(self, msg):
        self.pose = msg
    
    def control_loop(self):
        # Leitura do estado atual do robo

        # Usar variaveis self.joint_states e self.pose
        # Voce pode procurar na internet como seria a estrutura de mensagens
        # do tipo sensor_msgs/JointState e geometry_msgs/Pose
        
        # Computando acao de controle
        setpoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ##### Escrever acao de controle aqui

        # Criando mensagem
        msg = Float64MultiArray()
        msg.data = setpoint
        
        # Mandando sinal de atuação no robo
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)




def main(args=None):
    rclpy.init(args=args)

    denso_control = DensoControl()

    rclpy.spin(denso_control)

    denso_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
