import unittest
import numpy as np
from task_contact_surface import separate_contact_surface


class ContactSurfaceTests(unittest.TestCase):
    def fixture(self):
        x,y=np.meshgrid(np.linspace(-.25,.25,101),np.linspace(-.25,.25,101))
        table=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        x,y,z=np.meshgrid(np.linspace(.03,.09,15),np.linspace(-.03,.03,15),[.04,.05])
        target=np.c_[x.ravel(),y.ravel(),z.ravel()]
        xyz=np.vstack((table,target))
        uv=np.vstack((np.full((len(table),2),500.),np.full((len(target),2),50.)))
        return xyz,uv,len(table)

    def test_table_never_removed_and_metadata_is_preview_only(self):
        xyz,uv,ntable=self.fixture()
        background,target,report=separate_contact_surface(xyz,uv,[0,0,100,100],[0,0,.03])
        self.assertEqual(sum(abs(background[:,2])<1e-12),ntable)
        self.assertFalse(report['hardware_validated'])
        self.assertGreater(len(target),0)
        self.assertNotIn('approach',report['contact_stages'])

    def test_ambiguous_target_cells_remain_background(self):
        xyz,uv,_=self.fixture()
        suspect=xyz[-1:].copy();outside=np.array([[500,500]])
        background,_,_=separate_contact_surface(np.vstack((xyz,suspect)),np.vstack((uv,outside)),
                                                [0,0,100,100],[0,0,.03])
        self.assertTrue(np.any(np.all(background==suspect[0],axis=1)))

    def test_no_target_or_insufficient_table_fails(self):
        xyz,uv,ntable=self.fixture()
        with self.assertRaisesRegex(ValueError,'TARGET_NOT_SEPARATED'):
            separate_contact_surface(xyz[:ntable],uv[:ntable],[0,0,100,100],[0,0,.03])
        with self.assertRaisesRegex(ValueError,'TARGET_NOT_SEPARATED'):
            separate_contact_surface(xyz[-30:],uv[-30:],[0,0,100,100],[0,0,.03])

    def test_bad_coordinates_fail(self):
        xyz,uv,_=self.fixture();xyz[0,0]=np.nan
        with self.assertRaisesRegex(ValueError,'TARGET_NOT_SEPARATED'):
            separate_contact_surface(xyz,uv,[0,0,100,100],[0,0,.03])


if __name__=='__main__':unittest.main()
