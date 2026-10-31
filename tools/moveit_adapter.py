"""Bounded MoveIt service/action adapter for the isolated SG2 simulator only."""
import copy
import math
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import JointConstraint, Constraints, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener


def pose(x, y, z):
    value = Pose()
    value.position.x, value.position.y, value.position.z = float(x), float(y), float(z)
    value.orientation.w = 1.0
    return value


def ee_pose(x, y, z):
    value = pose(x, y, z)
    # SG2 gripper points along local-Z. Present its fingertip toward board+X.
    value.orientation.y = -math.sqrt(.5)
    value.orientation.w = math.sqrt(.5)
    return value


class SimMoveIt(Node):
    def __init__(self):
        if os.environ.get('ROS_DOMAIN_ID') != '64':
            raise RuntimeError('Only isolated simulation domain64 is supported')
        super().__init__('whiteboard_sim_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.state, self.state_receipt = None, 0.
        self.create_subscription(JointState, '/joint_states', self._state, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.service_clients = {name: self.create_client(cls, '/' + name) for name, cls in (
            ('compute_ik', GetPositionIK), ('plan_kinematic_path', GetMotionPlan),
            ('compute_cartesian_path', GetCartesianPath), ('apply_planning_scene', ApplyPlanningScene))}
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.sample_hook = lambda: None
        self.create_timer(.05, self._sample)
        for name, client in self.service_clients.items():
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('MoveIt service unavailable: ' + name)
        self.spin_until(lambda: self.state is not None and self.get_clock().now().nanoseconds > 0, 15)

    def _state(self, message):
        if len(message.name) == len(message.position) and all(math.isfinite(x) for x in message.position):
            self.state, self.state_receipt = message, time.monotonic()

    def _sample(self):
        self.sample_hook()

    def spin_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if condition():
                return
        raise TimeoutError('MoveIt operation exceeded wall timeout')

    def call(self, name, request, timeout=15):
        future = self.service_clients[name].call_async(request)
        self.spin_until(future.done, timeout)
        return future.result()

    def snapshot(self):
        if self.state is None or time.monotonic() - self.state_receipt > 1:
            raise RuntimeError('joint state is missing or stale')
        snapshot = copy.deepcopy(self.state)
        # Gazebo's settled zero joints can be +7e-14 above an upper bound of0.
        # Remove only sub-nanoradian zero noise; never clamp a meaningful error.
        snapshot.position = [0.0 if abs(v) < 1e-9 else v for v in snapshot.position]
        return snapshot

    def measured_tip(self):
        transform = self.buffer.lookup_transform('base_link', 'end_effector_r_link', Time())
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -.05 <= age <= .2:
            raise RuntimeError('TF stale or from another clock')
        p, q = transform.transform.translation, transform.transform.rotation
        return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]

    def setup_scene(self):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        board = CollisionObject()
        board.id, board.header.frame_id, board.operation = 'whiteboard', 'base_link', CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.02, .36, .30])
        board.primitives, board.primitive_poses = [box], [pose(.53, -.28, 1.05)]
        req.scene.world.collision_objects = [board]
        attached = AttachedCollisionObject()
        attached.link_name = 'end_effector_r_link'
        attached.object.id, attached.object.header.frame_id = 'sim_eraser', attached.link_name
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.045, .07, .04])]
        attached.object.primitive_poses = [pose(0, 0, -.06)]
        attached.touch_links = ['end_effector_r_link', 'arm_r_link7', 'camera_right_link',
            'gripper_r_rh_p12_rn_base', 'gripper_r_rh_p12_rn_l1', 'gripper_r_rh_p12_rn_l2',
            'gripper_r_rh_p12_rn_r1', 'gripper_r_rh_p12_rn_r2']
        req.scene.robot_state.attached_collision_objects = [attached]
        if not self.call('apply_planning_scene', req).success:
            raise RuntimeError('planning scene update refused')

    def plan_approach(self, target):
        ik = GetPositionIK.Request()
        ik.ik_request.group_name, ik.ik_request.ik_link_name = 'arm_r', 'end_effector_r_link'
        ik.ik_request.robot_state.joint_state = self.snapshot()
        ik.ik_request.robot_state.is_diff = True
        ik.ik_request.pose_stamped.header.frame_id = 'base_link'
        ik.ik_request.pose_stamped.pose = target
        ik.ik_request.avoid_collisions, ik.ik_request.timeout.sec = True, 2
        solution = self.call('compute_ik', ik)
        if solution.error_code.val != 1:
            # IK seed can be trapped at a redundant-arm joint boundary. Try a
            # zero-arm seed; the subsequent motion plan STILL starts at the
            # actual measured state and keeps collision checks enabled.
            seeded = copy.deepcopy(ik.ik_request.robot_state.joint_state)
            seeded.position = [0. if n.startswith('arm_r_joint') else v for n, v in zip(seeded.name, seeded.position)]
            ik.ik_request.robot_state.joint_state = seeded
            solution = self.call('compute_ik', ik)
        if solution.error_code.val != 1:
            raise RuntimeError('approach IK failed: ' + str(solution.error_code.val))
        req = GetMotionPlan.Request()
        plan = req.motion_plan_request
        plan.group_name, plan.pipeline_id = 'arm_r', 'ompl'
        plan.allowed_planning_time, plan.num_planning_attempts = 5., 5
        plan.max_velocity_scaling_factor = .15
        plan.max_acceleration_scaling_factor = .15
        plan.start_state.joint_state, plan.start_state.is_diff = self.snapshot(), True
        constraints = Constraints()
        for name, value in zip(solution.solution.joint_state.name, solution.solution.joint_state.position):
            if name.startswith('arm_r_joint'):
                constraints.joint_constraints.append(JointConstraint(joint_name=name, position=value,
                    tolerance_above=.005, tolerance_below=.005, weight=1.))
        plan.goal_constraints = [constraints]
        # Retry a failed planning service response. The vendor planning pipeline
        # keeps its own validation adapters; this is not an independent validator.
        for _ in range(4):
            response = self.call('plan_kinematic_path', req, timeout=40).motion_plan_response
            if response.error_code.val == 1:
                return response.trajectory
        raise RuntimeError('approach planning failed: ' + str(response.error_code.val))

    def cartesian(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id, req.group_name, req.link_name = 'base_link', 'arm_r', 'end_effector_r_link'
        req.start_state.joint_state, req.start_state.is_diff = self.snapshot(), True
        req.waypoints = waypoints
        req.max_step, req.avoid_collisions = .005, True
        req.revolute_jump_threshold, req.prismatic_jump_threshold = .35, .02
        req.max_velocity_scaling_factor = .10
        req.max_acceleration_scaling_factor = .10
        req.cartesian_speed_limited_link, req.max_cartesian_speed = 'end_effector_r_link', .025
        result = self.call('compute_cartesian_path', req, timeout=30)
        if result.error_code.val != 1 or result.fraction < .999:
            raise RuntimeError(f'Cartesian path incomplete: code={result.error_code.val}, fraction={result.fraction}')
        return result.solution, result.fraction

    def execute(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('trajectory execution action unavailable')
        future = self.execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory, controller_names=['arm_r_controller']))
        self.spin_until(future.done, 10)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('execution rejected')
        result_future = handle.get_result_async()
        try:
            self.spin_until(result_future.done, 120)
        except BaseException:
            # A failed sampling/visual service must not throw again while the
            # cancellation response is being processed.
            self.sample_hook = lambda: None
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done, 5)
            raise
        result = result_future.result()
        if result.status != 4 or result.result.error_code.val != 1:
            raise RuntimeError(f'execution failed: status={result.status}, code={result.result.error_code.val}')
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=.05)


if __name__ == '__main__':
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute-sim', action='store_true')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = SimMoveIt()
    node.setup_scene()
    trajectory = node.plan_approach(ee_pose(.395, -.38, 1.10))
    print('Approach plan points:', len(trajectory.joint_trajectory.points), flush=True)
    if args.execute_sim:
        node.execute(trajectory)
        trajectory, fraction = node.cartesian([ee_pose(.435, -.38, 1.10), ee_pose(.435, -.18, 1.10)])
        print('Wipe plan fraction:', fraction, flush=True)
        node.execute(trajectory)
        print('Measured end:', node.measured_tip(), flush=True)
    node.destroy_node()
    rclpy.shutdown()
