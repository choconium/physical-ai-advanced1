#!/usr/bin/env python3
"""Read-only simulation workspace/IK probe; never sends trajectories."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def main():
    rclpy.init(args=[])
    node = Node('whiteboard_workspace_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = []
    node.create_subscription(JointState, '/joint_states', lambda m: state.append(m), 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(GetPositionIK, '/compute_ik')
    if not client.wait_for_service(timeout_sec=30):
        raise RuntimeError('MoveIt /compute_ik not available')
    deadline = time.monotonic() + 10
    while not state and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=.02)
    frames = {}
    for name in ('arm_base_link', 'end_effector_r_link', 'end_effector_l_link'):
        tf = buffer.lookup_transform('base_link', name, Time())
        p, q = tf.transform.translation, tf.transform.rotation
        frames[name] = {'xyz': [p.x, p.y, p.z], 'xyzw': [q.x, q.y, q.z, q.w]}
    trials = []
    for xyz in ([.40,-.28,.9],[.45,-.28,1.05],[.5,-.28,1.1],[.5,-.35,1.25],[.35,-.4,1.0]):
        for q in ([0.,0.,0.,1.], [0.,.70710678,0.,.70710678], [0.,1.,0.,0.]):
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm_r'
            req.ik_request.ik_link_name = 'end_effector_r_link'
            req.ik_request.robot_state.joint_state = state[-1]
            req.ik_request.robot_state.is_diff = True
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = 'base_link'
            pose = req.ik_request.pose_stamped.pose
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            req.ik_request.timeout.sec = 1
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            result = future.result()
            if result is None:
                raise RuntimeError('IK response timeout')
            trials.append({'xyz': xyz, 'q': q, 'error_code': result.error_code.val,
                           'solution': dict(zip(result.solution.joint_state.name, result.solution.joint_state.position)) if result.error_code.val == 1 else {}})
    payload = {'frames': frames, 'trials': trials}
    Path('artifacts/workspace_probe.json').write_text(json.dumps(payload, indent=2))
    print(json.dumps({'frames': frames, 'reachable': [t for t in trials if t['error_code'] == 1]}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
