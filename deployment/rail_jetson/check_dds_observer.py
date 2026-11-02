"""Test command-source attribution on isolated loopback DDS174 only."""
import json
import os
import time
import xml.etree.ElementTree as ET

from bundle import ROOT
from dds_authority import DdsAuthority


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '174' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This test requires localhost DDS174')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        raise RuntimeError('This test requires the deployed Cyclone runtime')
    config = ET.fromstring(os.environ['CYCLONEDDS_URI'])
    # Foxy appends the legacy loopback option for ROS_LOCALHOST_ONLY=1.
    if config.findall('.//NetworkInterface') or config.findtext('.//NetworkInterfaceAddress') != 'lo':
        raise RuntimeError('This test requires explicit loopback-only DDS configuration')
    if config.findtext('.//AllowMulticast') != 'false':
        raise RuntimeError('Multicast must be disabled for this test')
    import rclpy
    from unitree_hg.msg import LowCmd
    rclpy.init()
    node = rclpy.create_node('isolated_source_attribution_test')
    observer = DdsAuthority(ROOT)
    try:
        if node.count_publishers('/arm_sdk'):
            raise RuntimeError('Isolated test domain already occupied')
        pub = node.create_publisher(LowCmd, '/arm_sdk', 10)
        end = time.monotonic() + 8.
        while time.monotonic() < end:
            pub.publish(LowCmd())
            rclpy.spin_once(node, timeout_sec=.05)
        gids = {bytes(p.endpoint_gid).hex()[:32] for p in node.get_publishers_info_by_topic('/arm_sdk')
                if p.node_name == node.get_name()}
        state = observer.snapshot()['discovery']
        assert len(gids) == 1 and set(state['messages']) == gids, state
        assert sum(state['messages'].values()) >= 5 and state['unknown_messages'] == 0, state
        print(json.dumps(dict(passed=True, domain=174, localhost_only=True,
                              attributed_messages=state['messages'], hardware_commands_sent=0), indent=2))
    finally:
        observer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
