"""Stationarity and freshness gates can be checked without ROS or hardware."""
import copy
import unittest

from capture_workbench_start_pose import validate_window


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.samples=[]
        for i in range(60):
            self.samples.append(dict(t=i*.04,tick=i,mode_machine=6,mode_pr=0,
                q=[0.]*29,dq=[0.]*29,vol=[52.]*13+[0.,0.]+[52.]*14,
                mode=[1]*13+[0,0]+[1]*14))

    def test_locked_waist_slots_are_not_required(self):
        self.assertEqual(validate_window(self.samples,.02)['samples'],60)

    def test_stale_and_replayed_samples_rejected(self):
        with self.assertRaises(ValueError):validate_window(self.samples,.3)
        for sample in self.samples:sample['tick']=1
        with self.assertRaises(ValueError):validate_window(self.samples,.02)

    def test_moving_or_inactive_arm_rejected(self):
        for field,value in (('q',.04),('dq',.2),('vol',0.)):
            samples=copy.deepcopy(self.samples)
            samples[-1][field][22]=value
            with self.subTest(field=field),self.assertRaises(ValueError):validate_window(samples,.02)

    def test_parallel_mode_and_nonfinite_rejected(self):
        self.samples[-1]['mode_pr']=1
        with self.assertRaises(ValueError):validate_window(self.samples,.02)
        self.samples[-1]['mode_pr']=0
        self.samples[-1]['q'][22]=float('nan')
        with self.assertRaises(ValueError):validate_window(self.samples,.02)


if __name__=='__main__':unittest.main()
