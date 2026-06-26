import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from abv_msgs.srv import AbvControlAction
from abv_msgs.msg import AbvVec3

@dataclass
class ControlContext:
    """Context for control policy computation. Includes current state, goal, and error.
        ROS2 message is converted to this internal type on reciept"""
    
    pose:     list[float]
    velocity: list[float]
    goal:     list[float]
    error:    list[float]
    
class PIDController:
    """Simple PID controller for testing interface to abv_controller software 
        running the feedback control loop"""
        
    def __init__(self):
        
        self._kp = [5.0, 5.0, 5.0]
        self._ki = [0.0, 0.0, 0.0]
        self._kd = [15, 15, 15]

        self._integral   = [0.0, 0.0, 0.0]
        self._prev_error = [0.0, 0.0, 0.0]
        self._prev_time  = None

    def compute(self, ctx: ControlContext, now) -> list[float]:
        if self._prev_time is None:
            self._prev_time = now
            return [0.0, 0.0, 0.0]

        dt = (now - self._prev_time).nanoseconds / 1e9
        self._prev_time = now

        if dt <= 0.0:
            return [0.0, 0.0, 0.0]

        action = [0.0, 0.0, 0.0]
        for i in range(3):
            self._integral[i]  += ctx.error[i] * dt
            derivative          = (ctx.error[i] - self._prev_error[i]) / dt
            action[i]           = (self._kp[i] * ctx.error[i] +
                                   self._ki[i] * self._integral[i] +
                                   self._kd[i] * derivative)
            
            self._prev_error[i] = ctx.error[i]

        return action

    def reset(self):
        self._integral   = [0.0, 0.0, 0.0]
        self._prev_error = [0.0, 0.0, 0.0]
        self._prev_time  = None
       
class RLController:

    def __init__(self):
        # TODO: load model, warm start torch?
        pass

    def compute(self, ctx: ControlContext, now) -> list[float]: 
        # TODO: convert context to state representation, run through model, convert output to action
        pass

class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('abv_rl')

        self._controller = PIDController()

        self.create_service(
            AbvControlAction,
            'abv/control_action',
            self._handle_request
        )

        self.get_logger().info('Policy node ready')

    def _handle_request(self, request, response):
        
        ctx = ControlContext(
            pose     = [request.pose.x, request.pose.y, request.pose.yaw],
            velocity = [request.vel.x, request.vel.y, request.vel.yaw],
            goal     = [request.goal.x, request.goal.y, request.goal.yaw],
            error    = [request.error.x, request.error.y, request.error.yaw]
        )
        
        action = self._controller.compute(ctx, self.get_clock().now())

        response.action.x   = action[0]
        response.action.y   = action[1]
        response.action.yaw = action[2]
        
        response.is_global = True

        return response


def main():
    rclpy.init()
    node = RLPolicyNode()
    rclpy.spin(node)
    rclpy.shutdown()