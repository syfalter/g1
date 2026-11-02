"""Per-client SDK DDS configuration; never mutates the controller environment."""
import ipaddress
import os
import xml.etree.ElementTree as ET


def sdk_environment(peer_address=None):
    environment=dict(os.environ)
    environment['LD_LIBRARY_PATH']='/home/syf/unitree_sdk2/thirdparty/lib/x86_64'
    if peer_address is not None:
        peer=str(ipaddress.ip_address(peer_address))
        config=environment.get('CYCLONEDDS_URI','')
        if not config.lstrip().startswith('<'):
            raise RuntimeError('SDK peer requires an explicit inline CycloneDDS configuration')
        root=ET.fromstring(config)
        domain=root.find('Domain')
        if domain is None:raise RuntimeError('SDK DDS configuration lacks Domain')
        discovery=domain.find('Discovery')
        if discovery is None:discovery=ET.SubElement(domain,'Discovery')
        # RPC clients use ephemeral receive ports, independent of ROS participants.
        index=discovery.find('ParticipantIndex')
        if index is None:index=ET.SubElement(discovery,'ParticipantIndex')
        index.text='none'
        peers=discovery.find('Peers')
        if peers is None:peers=ET.SubElement(discovery,'Peers')
        if not any(item.get('Address')==peer for item in peers.findall('Peer')):
            ET.SubElement(peers,'Peer',Address=peer)
        environment['CYCLONEDDS_URI']=ET.tostring(root,encoding='unicode')
    return environment
