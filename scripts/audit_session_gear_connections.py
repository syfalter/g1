"""Offline session inventory and gear reversal endpoint/hand/clock audit."""
import copy

import numpy as np
from scipy.spatial.transform import Rotation

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, encode, checked,
    context_for, write_json, OfflineModel, parameters_from_values, time_seconds)
from task_composition_preview import concatenate, materialize
from export_trajectory_only_candidates import imported, checkpoints_test
from automatic_recovery import reversed_path

OUT = ROOT / 'data/rail_deployment/session_inventory_20260929'
STAGE = ROOT / 'data/rail_deployment/trajectory_only_20260929'


def boundary(a, ai, b, bi, model, context):
    if list(a.joint_names) != list(b.joint_names):
        raise ValueError('Joint order mismatch')
    pa, pb = a.points[ai], b.points[bi]
    differences = {key:float(np.max(np.abs(np.asarray(getattr(pa, key)) - getattr(pb, key))))
                   for key in ('positions','velocities','accelerations')}
    stationary = max(abs(x) for point in (pa,pb) for field in ('velocities','accelerations')
                     for x in getattr(point,field))
    poses = model.batch_transforms(['right_wrist_yaw_link','right_index_tcp'], context,
        a.joint_names,np.array([pa.positions,pb.positions]))
    fk = {}
    for name, frames in poses.items():
        fk[name] = dict(position_mm=float(np.linalg.norm(frames[0,:3,3]-frames[1,:3,3])*1000.),
            orientation_deg=float(np.rad2deg(Rotation.from_matrix(frames[0,:3,:3].T @ frames[1,:3,:3]).magnitude())))
    passed = max(differences.values()) <= 1e-7 and stationary <= 1e-7
    if not passed:
        raise ValueError('Gear endpoint does not join C2: '+str(differences))
    return dict(max_joint_difference=differences, max_boundary_velocity_or_acceleration=float(stationary),
                fk_same_hand_posture=fk, stationary_c2_passed=passed)


def main():
    model = OfflineModel(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    base = read(BASE/'bundle.json')
    report = read(STAGE/'gear_roundtrip_lower_upper_report.json')
    bounded = load(ROOT/report['preview'],model)
    payload = bounded['variants']['flow']['payload']
    original = load(ROOT/base['payload']['actions']['gear_roundtrip_lower_upper']['source'],model)
    reverse_report = read(STAGE/'reverse_report.json')
    reverse = load(ROOT/reverse_report['preview'],model)['variants']['flow']['payload']['units']['ul']['trajectory']
    expected = reversed_path(original['variants']['flow']['payload']['units']['lu']['trajectory'])
    if reverse != expected:
        raise ValueError('Existing fast reversal is not the pinned lift reversed')
    params = parameters_from_values(base['payload']['actions']['gear_roundtrip_lower_upper']['plan']['planning_config']['values'])
    context = context_for(payload,'lu')
    protocol = imported('session_audit_gear_protocol',STAGE/'jetson_readonly/deployment/rail_jetson/gear_protocol.py')
    progress = imported('session_audit_gear_progress',STAGE/'jetson_readonly/deployment/rail_jetson/gear_progress.py')
    bounded_library = protocol.GearLibrary(read(STAGE/'data_only/config/gear_protocol.json'),
                                            read(STAGE/'data_only/config/selected_bundle.json'))
    pairs = [('lu_end_to_reverse_start','lu',-1,'ul',0),
             ('lu_end_to_upper_exit','lu',-1,'exit_upper',0),
             ('upper_entry_to_reverse','entry_upper',-1,'ul',0),
             ('upper_entry_to_lu_end','entry_upper',-1,'lu',-1),
             ('upper_exit_to_ready_entry','exit_upper',-1,'entry_upper',0),
             ('reverse_end_to_lu_start','ul',-1,'lu',0),
             ('reverse_end_to_lower_exit','ul',-1,'exit_lower',0),
             ('lower_entry_to_lu','entry_lower',-1,'lu',0),
             ('both_ready_entries','entry_lower',0,'entry_upper',0),
             ('both_ready_exits','exit_lower',-1,'exit_upper',-1)]
    scenarios = {}
    for name, spec in (('bounded',payload['units']['ul']['trajectory']),
                       ('existing_fast_reverse_9_099693',reverse),
                       ('reverse_latest_lift_8_963197',reversed_path(payload['units']['lu']['trajectory']))):
        units = {k:decode(v['trajectory']) for k,v in payload['units'].items()}
        units['ul'] = decode(spec)
        units['cycle_lower'] = concatenate([units['lu'],units['ul']])
        units['cycle_upper'] = concatenate([units['ul'],units['lu']])
        evidence = {label:boundary(units[a],ai,units[b],bi,model,context) for label,a,ai,b,bi in pairs}
        limits = {k:checked(tr,model,params) for k,tr in units.items()}
        # In-memory protocol simulation only. These modified units are never
        # exported as a production library with the old release bindings.
        library = copy.deepcopy(bounded_library)
        library.units = {k:encode(v) for k,v in units.items()}
        library._blocks = {}
        clocks = checkpoints_test(library,progress.GearProgress,bounded_library)
        sequences = {}
        for pre in ('NONE','EXTEND_DOWNLOCK'):
            for mode in ('FULL_CYCLE','FINAL_RETRACT_ONLY'):
                request = dict(count=1,preAction=pre,cycleMode=mode)
                program = list(library.program(request))
                motion = [decode(v['trajectory']) for v in program if v['kind']=='recorded']
                composed = concatenate(motion)
                checked(composed,model,params)
                hands = [dict(posture=v['label'],duration=v['duration'],sequence=v['finger_sequence'],
                              target_q=v['target_q']) for v in program if v['kind']=='hand']
                if [h['sequence'] for h in hands] != ['thumb_then_four','four_then_thumb','thumb_then_four']:
                    raise ValueError('Changed gear hand ordering')
                sequences[pre+'/'+mode] = dict(request=request,describe=library.describe(request),
                    hand_events=hands,all_motion_boundaries_c2=True,
                    motion_seconds=time_seconds(composed.points[-1]),
                    upper_dwell_added=False)
        scenarios[name] = dict(seconds={k:time_seconds(v.points[-1]) for k,v in units.items()},
            boundaries=evidence,continuous_limits=limits,progress_cases=clocks,sequences=sequences,
            boundary_max_rad=max(v['max_joint_difference']['positions'] for v in evidence.values()),
            production_bindings_rebuilt=False,physical_verified=False)
    result = dict(source_release_sha256=base['sha256'],gear_bounded_snapshot=report['preview'],
        existing_reverse_snapshot=reverse_report['preview'],scenarios=scenarios,
        reverse_contact_diagnostic={k:reverse_report[k] for k in (
            'sampled_bidirectional_distance_mm','sampled_nearest_orientation_deg','rejection_gates')},
        physical_verified=False,hardware_commands_sent=0,
        warnings=['Stationary matching endpoints do not prove reversed contact/grasp safety',
                  'Open/grip finger FK differs intentionally; wrist joint endpoints match',
                  'A reversed UL ends at LOWER and must use exit_lower, not exit_upper',
                  'No extra upper dwell exists in the business cycle; adding it changes timing',
                  'Activating a reversal requires rebuilding both cycles, full actions, bindings and progress checks'])
    write_json(OUT/'gear_connections.json',result)
    for name,value in scenarios.items():
        print(name,'LU',value['seconds']['lu'],'UL',value['seconds']['ul'],
              'MAX_Q_DELTA',value['boundary_max_rad'],'PROGRESS_CASES',len(value['progress_cases']),flush=True)


if __name__=='__main__':
    main()
