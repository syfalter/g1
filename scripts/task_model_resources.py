"""Local URDF resources and verified archive relocation; no robot I/O."""
import json
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from task_plan_archive import digest


def resolve_mesh(reference, urdf):
    if not isinstance(reference,str) or not reference:
        raise ValueError('Missing mesh reference')
    uri=urlparse(reference)
    if uri.scheme:
        if uri.query or uri.fragment:raise ValueError('Unexpected mesh URI query/fragment')
        if uri.scheme=='package':
            from ament_index_python.packages import get_package_share_directory
            root=Path(get_package_share_directory(uri.netloc)).resolve()
            path=(root/unquote(uri.path.lstrip('/'))).resolve()
            if not path.is_relative_to(root):raise ValueError('Mesh escapes package')
            return path
        if uri.scheme!='file' or uri.netloc not in ('','localhost'):
            raise ValueError('Unsupported mesh URI: '+reference)
        path=Path(unquote(uri.path))
        if not path.is_absolute():raise ValueError('File mesh URI must be absolute')
    else:
        path=Path(reference)
        if not path.is_absolute():path=Path(urdf).parent/path
    return path.resolve()


def archived_model_tree(directory, *, require_snapshots=True):
    """Call after archive verification; never rewrite the original model."""
    directory=Path(directory).resolve()
    source=directory/'context/robot.urdf'
    tree=ET.parse(source)
    metadata=directory/'plan/offline_provenance.json'
    if not metadata.is_file() and not require_snapshots:
        for mesh in tree.iter('mesh'):
            mesh.set('filename',resolve_mesh(mesh.attrib['filename'],source).as_uri())
        return tree,{}
    resources=json.loads(metadata.read_text()).get('mesh_resources',[]) if metadata.is_file() else []
    mapping={record['reference']:record for record in resources}
    if len(mapping)!=len(resources):raise ValueError('Duplicate archived mesh reference')
    hashes={}
    for mesh in tree.iter('mesh'):
        reference=mesh.attrib['filename'];record=mapping.get(reference)
        if record is None:raise ValueError('Missing archived mesh resource: '+reference)
        saved=(directory/'plan'/record['snapshot']).resolve()
        if not saved.is_relative_to(directory) or digest(saved)!=record['sha256']:
            raise ValueError('Archived mesh resource changed')
        hashes[reference]=record['sha256']
        mesh.set('filename',saved.as_uri())
    return tree,hashes


def verify_live_meshes(directory, robot_urdf):
    directory=Path(directory).resolve();robot_urdf=Path(robot_urdf).resolve()
    if not (directory/'plan/offline_provenance.json').is_file():return
    _,hashes=archived_model_tree(directory)
    if robot_urdf==directory/'context/robot.urdf':return
    for reference,expected in hashes.items():
        try:actual=digest(resolve_mesh(reference,robot_urdf))
        except OSError as error:raise ValueError('Live mesh unavailable: '+reference) from error
        if actual!=expected:raise ValueError('Live mesh differs from archived mesh: '+reference)
