#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time
import sys


def activate_node(node_name, node, timeout=5.0):
    """Activate a lifecycle node"""
    client = node.create_client(ChangeState, f'{node_name}/change_state')
    
    if not client.wait_for_service(timeout_sec=timeout):
        print(f'⚠️  Service not available for {node_name}')
        return False
    
    # Configure
    print(f'Configuring {node_name}...')
    request = ChangeState.Request()
    request.transition.id = Transition.TRANSITION_CONFIGURE
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    
    result = future.result()
    if result is None:
        print(f'❌ Failed to configure {node_name}')
        return False
    
    if not result.success:
        print(f'⚠️  Configure returned False for {node_name}')
        return False
    
    time.sleep(0.5)
    
    # Activate
    print(f'Activating {node_name}...')
    request = ChangeState.Request()
    request.transition.id = Transition.TRANSITION_ACTIVATE
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    
    result = future.result()
    if result is None:
        print(f'❌ Failed to activate {node_name}')
        return False
    
    if not result.success:
        print(f'⚠️  Activate returned False for {node_name}')
        return False
    
    print(f'✅ Successfully activated {node_name}')
    return True


def main():
    rclpy.init()
    node = Node('nav2_activator')
    
    # List of Nav2 lifecycle nodes to activate
    nav2_nodes = [
        '/map_server',
        '/planner_server',
        '/controller_server',
        '/bt_navigator',
        '/global_costmap/global_costmap',
        '/local_costmap/local_costmap',
    ]
    
    print('=' * 60)
    print('Activating Nav2 lifecycle nodes...')
    print('=' * 60)
    
    success_count = 0
    for nav_node in nav2_nodes:
        try:
            if activate_node(nav_node, node):
                success_count += 1
        except Exception as e:
            print(f'❌ Error activating {nav_node}: {e}')
    
    print('=' * 60)
    print(f'Activated {success_count}/{len(nav2_nodes)} nodes')
    print('=' * 60)
    
    node.destroy_node()
    rclpy.shutdown()
    
    if success_count == len(nav2_nodes):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()

