#!/usr/bin/env python3
"""Change one motor's remaining effectiveness; preserve other motors' limits."""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from controller.srv import SetMotorLimits
from std_msgs.msg import Float64MultiArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('motor', type=int, help='1..8, same numbering as the paper')
    parser.add_argument('effectiveness', type=float, help='0=complete loss; 1=restore; e.g. .333=partial loss')
    args = parser.parse_args()
    if not 1 <= args.motor <= 8 or not 0 <= args.effectiveness <= 1:
        parser.error('motor must be 1..8 and effectiveness must be 0..1')
    rclpy.init()
    node = Node('set_motor_failure')
    received = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(Float64MultiArray, '/omnicopter/motor_limits',
                                            lambda message: received.append(list(message.data)), qos)
    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(name + ' unavailable')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError(name + ' timed out')
            return future.result()
        finally:
            node.destroy_client(client)
    try:
        end = time.monotonic() + 10
        while not received and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if not received or len(received[-1]) != 16:
            raise RuntimeError('No valid motor-limit vector received')
        request = GetParameters.Request()
        request.names = ['thrust_max']
        nominal = call(GetParameters, '/failure_controller/get_parameters', request).values[0].double_array_value
        if len(nominal) != 8:
            raise RuntimeError('Controller nominal thrust limits unavailable')
        values = received[-1]
        i = args.motor-1
        values[i] = 0.
        values[8+i] = nominal[i]*args.effectiveness
        request = SetMotorLimits.Request()
        request.lower, request.upper = values[:8], values[8:]
        response = call(SetMotorLimits, '/failure_controller/set_motor_limits', request)
        if not response.success:
            raise RuntimeError(response.message)
        print('Motor %d (rotor_%d): remaining effectiveness %.6g, upper limit %.6g N' %
              (args.motor, i, args.effectiveness, values[8+i]))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
