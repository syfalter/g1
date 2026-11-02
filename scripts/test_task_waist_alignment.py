import unittest
import json
import tempfile
from pathlib import Path
from align_saved_waist import trajectory_for, load_collision_context, arm_excursion_for
from task_plan_archive import digest, verify
from task_session import ARM_NAMES, ARMS
from task_trajectory import samples, validate_waist_envelope


class WaistAlignmentTests(unittest.TestCase):
    def test_arm_excursion_returns_to_hold_without_waist_command(self):
        q=dict.fromkeys(ARM_NAMES,.2)
        trajectory,duration=arm_excursion_for(q,q,.05235987756)
        self.assertEqual(trajectory.joint_names,list(ARMS['left']))
        self.assertGreaterEqual(duration,5.)
        for point in (trajectory.points[0],trajectory.points[-1]):
            self.assertEqual(list(point.positions),[.2]*7)
            self.assertTrue(all(abs(v)<1e-10 for v in point.velocities+point.accelerations))
        for positions in samples(trajectory,.06,.12):
            for axis in (0,1,2,4,5,6):self.assertAlmostEqual(positions[axis],.2)
            self.assertTrue(.2-1e-10 <= positions[3] <= .25235987757)
        for delta in (0.,.11,float('nan')):
            with self.assertRaises(ValueError):arm_excursion_for(q,q,delta)

    def test_geometry_import_does_not_authorize_stored_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'plan').mkdir();(root/'context').mkdir()
            model=root/'context/robot.urdf';model.write_text('<robot/>')
            scene=dict(profile='steep_straight',base_radius_m=.025,base_height_m=.02,
                       arm_body_collision_checked=True,base_top_pelvis_m=[.3,0.,.1])
            plan=root/'plan/plan.json'
            plan.write_text(json.dumps(dict(arm='left',kind='task',segments=[],contact_scene=scene)))
            (root/'archive_manifest.json').write_text(json.dumps(dict(sha256={
                'plan/plan.json':digest(plan),'context/robot.urdf':digest(model)})))
            self.assertEqual(load_collision_context(root,model),scene)
            with self.assertRaises(ValueError):verify(root)
            other=root.parent/(root.name+'_other.urdf')
            try:
                other.write_text('<different/>')
                with self.assertRaisesRegex(ValueError,'model differs'):
                    load_collision_context(root,other)
            finally:other.unlink(missing_ok=True)
            plan.write_text('{}')
            with self.assertRaisesRegex(ValueError,'checksum mismatch'):
                load_collision_context(root,model)

    def test_only_yaw_reference_changes_with_smooth_endpoints(self):
        q=dict.fromkeys(ARM_NAMES,.2);q['waist_yaw_joint']=-.34
        held=dict(q);held[ARMS['left'][0]]+=.001
        trajectory,duration=trajectory_for(q,held,-.39883486758863934)
        self.assertGreaterEqual(duration,2.)
        for point in trajectory.points:
            self.assertEqual(list(point.positions[1:]),[held[n] for n in ARMS['left']])
        for point in (trajectory.points[0],trajectory.points[-1]):
            self.assertTrue(all(abs(v)<1e-10 for v in point.velocities+point.accelerations))
        self.assertAlmostEqual(trajectory.points[-1].positions[0],-.39883486758863934)
        validate_waist_envelope(trajectory,-.59,.59,.06,.12)
        self.assertGreater(len(list(samples(trajectory,.06,.12))),100)

    def test_large_or_nonfinite_move_rejected(self):
        q=dict.fromkeys(ARM_NAMES,.2);q['waist_yaw_joint']=-.34
        for target in (.1,float('nan')):
            with self.assertRaises(ValueError):trajectory_for(q,q,target)

    def test_hold_offset_does_not_create_initial_reverse_motion(self):
        q=dict.fromkeys(ARM_NAMES,.2);q['waist_yaw_joint']=-.006
        held=dict(q);held['waist_yaw_joint']=-.004
        trajectory,_=trajectory_for(q,held,.08)
        positions=[p.positions[0] for p in trajectory.points]
        self.assertEqual(positions[0],held['waist_yaw_joint'])
        self.assertTrue(all(b>=a for a,b in zip(positions,positions[1:])))
        held['waist_yaw_joint']=.02
        with self.assertRaisesRegex(ValueError,'settle'):trajectory_for(q,held,.08)


if __name__=='__main__':unittest.main()
