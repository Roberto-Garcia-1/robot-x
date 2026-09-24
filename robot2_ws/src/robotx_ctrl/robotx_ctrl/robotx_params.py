#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class RobotParamSrv(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Declaración de parametros
        self.declare_parameter(
            name='sample_timer',
            value=0.0,
            descriptor=ParameterDescriptor(
                description="Configuración del timer de muestreo"
            )
        )
        self.declare_parameters(
            namespace='movil',
            parameters=[
                ('lin_vel', 0.1),
                ('ang_vel', 0.04),
                ('label_sample', rclpy.Parameter.Type.STRING),
                ('int_array', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('pid_const', [0.5, 1.0, 1000.0])
            ]
        )

        self.add_on_set_parameters_callback(self._on_parameter_callback)

    def _on_parameter_callback(self, params):
        success=True
        for param in params:
            if param.name == 'sample_timer':
                if param.value < 0.0:
                    success = False
                    self.get_logger().warning(f"El parámetro '{param.name}' no puede ser negativo (valor={param.value}).")
            else:
                self.get_logger().info(f"El parametro {param.name} no es monitoreado")

        result_msg = SetParametersResult()
        result_msg.reason = "Intercepted for validation"
        result_msg.successful = success

        return result_msg


def init_node(args=None):
    rclpy.init(args=args)
    parm_srv=RobotParamSrv('robotx_paramsrv_node')
    rclpy.spin(parm_srv)
    rclpy.shutdown()

if __name__ == '__main__':
    init_node()
