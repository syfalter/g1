import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from task_model_resources import archived_model_tree,resolve_mesh,verify_live_meshes
from task_plan_archive import digest,write_json


class ModelResourceTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        self.archive=self.root/'archive with spaces'
        (self.archive/'context').mkdir(parents=True)
        (self.archive/'plan/model_resources').mkdir(parents=True)
        self.external=self.root/'original mesh.stl';self.external.write_bytes(b'mesh fixture')
        self.reference=self.external.as_uri()
        self.saved=self.archive/'plan/model_resources/snapshot.stl'
        self.saved.write_bytes(self.external.read_bytes())
        self.model=self.root/'robot.urdf'
        self.model.write_text(f'<robot><link name="base"><visual><geometry>'
            f'<mesh filename="{self.reference}"/></geometry></visual></link></robot>')
        (self.archive/'context/robot.urdf').write_bytes(self.model.read_bytes())
        self.metadata=dict(mesh_resources=[dict(reference=self.reference,
            snapshot='model_resources/snapshot.stl',sha256=digest(self.saved))])
        self.provenance=self.archive/'plan/offline_provenance.json'
        write_json(self.provenance,self.metadata)

    def test_structured_local_uri_resolution_and_relative_paths(self):
        self.assertEqual(resolve_mesh(self.reference,self.model),self.external)
        self.assertEqual(resolve_mesh('original mesh.stl',self.model),self.external)
        for reference in ('https://example.invalid/mesh.stl','file://remote/mesh.stl',
                          'file:relative.stl',self.reference+'?changed=1',''):
            with self.assertRaises(ValueError):resolve_mesh(reference,self.model)

    def test_relocation_preserves_source_and_uses_archived_bytes_after_source_removal(self):
        original=digest(self.archive/'context/robot.urdf')
        self.external.unlink()
        tree,hashes=archived_model_tree(self.archive)
        reference=next(tree.iter('mesh')).attrib['filename']
        self.assertEqual(resolve_mesh(reference,self.model),self.saved)
        self.assertEqual(hashes,{self.reference:digest(self.saved)})
        self.assertEqual(digest(self.archive/'context/robot.urdf'),original)
        verify_live_meshes(self.archive,self.archive/'context/robot.urdf')
        with self.assertRaisesRegex(ValueError,'Live mesh unavailable'):
            verify_live_meshes(self.archive,self.model)

    def test_changed_live_mesh_is_rejected_without_changing_archive(self):
        verify_live_meshes(self.archive,self.model)
        self.external.write_bytes(b'changed live geometry')
        with self.assertRaisesRegex(ValueError,'Live mesh differs'):
            verify_live_meshes(self.archive,self.model)
        archived_model_tree(self.archive)

    def test_missing_duplicate_escaped_or_changed_snapshot_is_rejected(self):
        for kind in ('missing','duplicate','escaped','checksum'):
            data=copy.deepcopy(self.metadata)
            if kind=='missing':data['mesh_resources']=[]
            elif kind=='duplicate':data['mesh_resources']*=2
            elif kind=='escaped':data['mesh_resources'][0]['snapshot']=str(self.external)
            else:data['mesh_resources'][0]['sha256']='0'*64
            write_json(self.provenance,data)
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                archived_model_tree(self.archive)

    def test_legacy_external_resources_remain_explicitly_supported(self):
        self.provenance.unlink()
        with self.assertRaisesRegex(ValueError,'Missing archived mesh'):
            archived_model_tree(self.archive)
        tree,hashes=archived_model_tree(self.archive,require_snapshots=False)
        self.assertEqual(hashes,{})
        self.assertEqual(next(tree.iter('mesh')).attrib['filename'],self.reference)


if __name__=='__main__':unittest.main()
