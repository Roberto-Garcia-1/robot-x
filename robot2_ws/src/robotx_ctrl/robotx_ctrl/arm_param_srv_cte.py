#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import ListParameters, GetParameters

class ArmParamCte(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.__param_names = []
        self.__params_dic = {}
        # Declaracion de las llamadas a servicios
        self.__get_params_client = self.create_client(
            GetParameters,
            '/arm_param_srv/get_parameters'
        )
        self.__list_param_client = self.create_client(
            ListParameters,
            '/arm_param_srv/list_parameters'
        )
        list_param_request = ListParameters.Request()
        list_param_request.prefixes.append('arm')
        list_param_request.depth = 0
        self.get_logger().info(f"Calling '/arm_param_srv/list_parameters': ListParameter.Request: prefixes: {list_param_request.prefixes}, depth: {list_param_request.depth}.")
        try:
            tries = 3
            while not self.__list_param_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available ({tries}), waiting again...')
                tries -= 1
                if tries == 0:
                    self.get_logger().warn(f'Service not available, Service unavailable.')
                    return
                
            self.get_logger().info("Calling '/arm_param_srv/list_parameters' async.")
            future = self.__list_param_client.call_async(request=list_param_request)
            future.add_done_callback(self.__on_list_param_srv_clbk)

        except Exception as e:
            self.get_logger().warn("service '/arm_param_srv/list_parameters' failed %r " % (e, )) 

    def __on_list_param_srv_clbk(self, future):
        try:
            list_params_result = future.result().result
            if list_params_result.names:
                get_parameters_request = GetParameters.Request()
                self.__param_names = get_parameters_request.names = list_params_result.names
                tries = 3
                while not self.__get_params_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'Service not available ({tries}), waiting again...')
                    tries -= 1
                    if tries == 0:
                        self.get_logger().warn(f'Service not available.')
                        return
                    
                self.get_logger().info("Calling service: '/arm_param_srv/get_parameters' async.")
                future = self.__get_params_client.call_async(request=get_parameters_request)
                future.add_done_callback(self.__on_get_params_srv_clbk)

        except Exception as e:
            self.get_logger().warn("On service '/arm_param_srv/list_parameters' callback failed %r " % (e, )) 

    def __on_get_params_srv_clbk(self, future):
        try:
            get_parameters_result = future.result()
            #print(get_parameters_result.values)

            arm_params_dic = self.__get_recursive_params(
                self.__param_names,
                get_parameters_result.values,
                prefix='arm.'
                )
            print(arm_params_dic)
            self.__params_dic = arm_params_dic
        except Exception as e:
            self.get_logger().warn("On service '/arm_param_srv/get_parameters' callback failed %r " % (e, )) 

    def __get_recursive_params(self, names, values, prefix: str = ''):
        result = {}
        indx = 0
        for param_name in names:
            if param_name.startswith(prefix):
                key_path = param_name[len(prefix):].strip('.')
                keys = key_path.split('.')
                current = result
                for k in keys[:-1]:
                    current = current.setdefault(k, {})
                current[keys[-1]] = self.__parse_parameter_value(values[indx])    
            indx += 1

        return result

    def __parse_parameter_value(self, parameter:ParameterValue):
        if parameter.type == ParameterType.PARAMETER_BOOL:
            return parameter.bool_value
        elif parameter.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return parameter.bool_array_value
        elif parameter.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return parameter.byte_array_value
        elif parameter.type == ParameterType.PARAMETER_DOUBLE:
            return parameter.double_value
        elif parameter.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return parameter.double_array_value.tolist()
        elif parameter.type == ParameterType.PARAMETER_INTEGER:
            return parameter.integer_value
        elif parameter.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return parameter.integer_array_value.tolist()
        elif parameter.type == ParameterType.PARAMETER_STRING:
            return parameter.string_value
        elif parameter.type == ParameterType.PARAMETER_STRING_ARRAY:
            return parameter.string_array_value
        else:
            return None


def init_node(args=None):
    try:
        rclpy.init(args=args)
        arm_param_cte_node = ArmParamCte('arm_param_cte')
        rclpy.spin(arm_param_cte_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Program finalized.')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    init_node()