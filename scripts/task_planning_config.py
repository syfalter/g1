"""Task snapshots and startup-only upper-body controller settings."""
from dataclasses import asdict, dataclass, fields, replace
import hashlib
import math
from pathlib import Path

import yaml
from task_stage_control import STAGE_GAIN_FLOAT_MAX

DEFAULT_CONFIG=Path(__file__).resolve().parents[1]/'config/task_planning.yaml'
MEASURED_VELOCITY_LIMIT=.5
TRACKING_VELOCITY_MARGIN=.1


@dataclass(frozen=True)
class PlanningParameters:
    radial_offset_mm: float=10.
    height_offset_mm: float=10.
    arc_center_toward_robot_mm: float=0.
    path_tolerance_mm: float=2.
    target_tolerance_mm: float=2.
    position_search_fraction: float=0.
    withdraw_height_mm: float=100.
    approach_via_center: bool=False
    approach_center_height_mm: float=120.
    approach_tilt_deg: float=30.
    approach_twist_deg: float=45.
    tilt_deg: float=5.
    twist_deg: float=10.
    step_deg: float=2.
    arc_yaw_gain: float=0.
    hand_back_yaw_deg: float=45.
    hand_back_tangent_xy: bool=False
    left_sweep_hand_back_yaw_deg: float=0.
    right_sweep_hand_back_yaw_deg: float=0.
    sweep_hand_back_min_deg: float=0.
    sweep_hand_back_max_deg: float=0.
    max_index_tilt_deg: float=45.
    planning_margin_deg: float=.1
    arc_segments: int=120
    line_points: int=60
    max_attempts: int=220
    budget_seconds: float=30.
    batch_trials: int=32
    batch_candidates: int=2
    complete_candidates: int=8
    small_step_fast_path: bool=True
    right_sweep_waist_bias: bool=False
    max_repairs: int=3
    backtrack_points: int=3
    contact_branch_search: bool=False
    contact_branch_attempts: int=6
    contact_search_seconds: float=180.
    contact_candidate_seconds: float=45.
    smooth_contact_path: bool=False
    smooth_insert_path: bool=False
    smooth_joint_path: bool=False
    smoothing_joint_deviation_deg: float=2.
    arm_velocity_rad_s: float=.4
    waist_velocity_rad_s: float=.35
    center_check_height_mm: float=50.
    endpoint_tolerance_deg: float=math.degrees(.025)
    endpoint_wait_seconds: float=5.
    stage_control_enabled: bool=False
    rotate_velocity_factor: float=1.
    withdraw_velocity_factor: float=1.
    rotate_proximal_kp: float=80.
    rotate_proximal_kd: float=4.5
    withdraw_proximal_kp: float=80.
    withdraw_proximal_kd: float=4.5
    stage_waist_kp: float=0.
    stage_waist_kd: float=0.
    demo_wrist_kp: float=0.
    demo_distal_wrist_kp: float=0.
    replay_shoulder_pitch_kp: float=-1.
    replay_shoulder_roll_kp: float=-1.
    replay_shoulder_yaw_kp: float=-1.
    replay_elbow_kp: float=-1.
    replay_wrist_roll_kp: float=-1.
    replay_wrist_pitch_kp: float=-1.
    replay_wrist_yaw_kp: float=-1.
    replay_waist_kp: float=-1.
    rail_replay_multiplier: float=1.
    rail_replay_fastest: bool=False
    rail_trim_idle_ends: bool=False
    rail_replay_smooth: bool=False
    rail_replay_cruise: bool=False
    left_shoulder_pitch_kp: float=55.
    left_shoulder_roll_kp: float=55.
    left_shoulder_yaw_kp: float=55.
    left_elbow_kp: float=55.
    left_wrist_roll_kp: float=40.
    left_wrist_pitch_kp: float=55.
    left_wrist_yaw_kp: float=55.
    left_shoulder_pitch_kd: float=3.
    left_shoulder_roll_kd: float=3.
    left_shoulder_yaw_kd: float=3.
    left_elbow_kd: float=3.
    left_wrist_roll_kd: float=3.5
    left_wrist_pitch_kd: float=3.
    left_wrist_yaw_kd: float=3.
    right_shoulder_pitch_kp: float=55.
    right_shoulder_roll_kp: float=55.
    right_shoulder_yaw_kp: float=55.
    right_elbow_kp: float=55.
    right_wrist_roll_kp: float=40.
    right_wrist_pitch_kp: float=55.
    right_wrist_yaw_kp: float=55.
    right_shoulder_pitch_kd: float=3.
    right_shoulder_roll_kd: float=3.
    right_shoulder_yaw_kd: float=3.
    right_elbow_kd: float=3.
    right_wrist_roll_kd: float=3.5
    right_wrist_pitch_kd: float=3.
    right_wrist_yaw_kd: float=3.
    waist_kp: float=60.
    waist_kd: float=4.5
    waist_ki: float=20.
    waist_integral_torque_limit_nm: float=1.2
    waist_integral_recovery_error_deg: float=math.degrees(.04)
    waist_integral_build_rate_nm_s: float=1.
    waist_integral_curve_scale: float=.25
    waist_integral_smooth_stiction: bool=False
    left_arm_integral_enabled: bool=True
    right_arm_integral_enabled: bool=False
    passive_arm_integral_enabled: bool=False
    arm_integral_recovery_error_deg: float=math.degrees(.04)
    arm_integral_hold_rate_nm_s: float=.25
    shoulder_pitch_ki: float=5.
    shoulder_roll_ki: float=5.
    shoulder_yaw_ki: float=3.
    elbow_ki: float=3.
    wrist_roll_ki: float=1.
    wrist_pitch_ki: float=1.
    wrist_yaw_ki: float=2.
    shoulder_pitch_integral_limit_nm: float=.6
    shoulder_roll_integral_limit_nm: float=.8
    shoulder_yaw_integral_limit_nm: float=.35
    elbow_integral_limit_nm: float=.4
    wrist_roll_integral_limit_nm: float=.15
    wrist_pitch_integral_limit_nm: float=.2
    wrist_yaw_integral_limit_nm: float=.25

    def orientation_policy(self):
        return dict(tilt_deg=self.tilt_deg,roll_deg=self.twist_deg,step_deg=self.step_deg)

    def arm_integral_parameters(self, side):
        if side not in ('left','right'):raise ValueError('Unknown arm side')
        return dict(arm_integral_enabled=getattr(self,side+'_arm_integral_enabled'),
            arm_integral_ki=[float(getattr(self,j+'_ki')) for j in ARM_INTEGRAL_JOINTS],
            arm_integral_limits_nm=[float(getattr(self,j+'_integral_limit_nm')) for j in ARM_INTEGRAL_JOINTS])

    def controller_parameters(self, side):
        result=self.arm_integral_parameters(side)
        other='right' if side=='left' else 'left'
        result.update(passive_arm_integral_enabled=self.passive_arm_integral_enabled and
                      getattr(self,other+'_arm_integral_enabled'),
                      arm_integral_recovery_error_deg=self.arm_integral_recovery_error_deg)
        result['arm_integral_hold_rate_nm_s']=self.arm_integral_hold_rate_nm_s
        for arm in ('left','right'):
            for gain in ('kp','kd'):
                result[arm+'_arm_'+gain]=[
                    float(getattr(self,arm+'_'+j+'_'+gain)) for j in ARM_INTEGRAL_JOINTS]
        result.update({key:(getattr(self,key) if isinstance(getattr(self,key),bool) else float(getattr(self,key)))
                       for key in SECTIONS['waist_controller']})
        result.update(max_velocity_rad_s=self.arm_velocity_rad_s,
            task_waist_velocity_limit_rad_s=self.waist_velocity_rad_s,
            max_measured_velocity_rad_s=MEASURED_VELOCITY_LIMIT)
        return result


ARM_INTEGRAL_JOINTS=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')
SECTIONS={
    'geometry':('radial_offset_mm','height_offset_mm','arc_center_toward_robot_mm','path_tolerance_mm','target_tolerance_mm',
                'position_search_fraction','withdraw_height_mm','approach_via_center','approach_center_height_mm'),
    'orientation':('tilt_deg','twist_deg','step_deg','arc_yaw_gain','hand_back_yaw_deg','hand_back_tangent_xy',
                   'left_sweep_hand_back_yaw_deg','right_sweep_hand_back_yaw_deg',
                   'sweep_hand_back_min_deg','sweep_hand_back_max_deg','max_index_tilt_deg','planning_margin_deg',
                   'approach_tilt_deg','approach_twist_deg'),
    'sampling':('arc_segments','line_points'),
    'search':('max_attempts','budget_seconds','batch_trials','batch_candidates','complete_candidates','small_step_fast_path',
              'right_sweep_waist_bias'),
    'repair':('max_repairs','backtrack_points','contact_branch_search','contact_branch_attempts','contact_search_seconds',
              'contact_candidate_seconds'),
    'smoothing':('smooth_contact_path','smooth_insert_path','smooth_joint_path','smoothing_joint_deviation_deg'),
    'motion':('arm_velocity_rad_s','waist_velocity_rad_s'),
    'localization':('center_check_height_mm',),
    'execution':('endpoint_tolerance_deg','endpoint_wait_seconds'),
    'stage_control':('stage_control_enabled','rotate_velocity_factor','withdraw_velocity_factor',
                     'rotate_proximal_kp','rotate_proximal_kd','withdraw_proximal_kp','withdraw_proximal_kd',
                     'stage_waist_kp','stage_waist_kd','demo_wrist_kp','demo_distal_wrist_kp'),
    'demo_replay':tuple('replay_'+j+'_kp' for j in ARM_INTEGRAL_JOINTS)+('replay_waist_kp',),
    'rail_replay':('rail_replay_multiplier','rail_replay_fastest','rail_trim_idle_ends','rail_replay_smooth','rail_replay_cruise'),
    'arm_pd':tuple(side+'_'+j+'_'+gain for side in ('left','right')
                   for gain in ('kp','kd') for j in ARM_INTEGRAL_JOINTS),
    'waist_controller':('waist_kp','waist_kd','waist_ki','waist_integral_torque_limit_nm',
                        'waist_integral_recovery_error_deg','waist_integral_build_rate_nm_s',
                        'waist_integral_curve_scale','waist_integral_smooth_stiction'),
    'arm_integral':('left_arm_integral_enabled','right_arm_integral_enabled',
                    'passive_arm_integral_enabled','arm_integral_recovery_error_deg','arm_integral_hold_rate_nm_s')+
        tuple(j+'_ki' for j in ARM_INTEGRAL_JOINTS)+tuple(j+'_integral_limit_nm' for j in ARM_INTEGRAL_JOINTS),
}
BOUNDS={
    'rail_replay_multiplier':(.1,10.),
    'rotate_velocity_factor':(.1,1.),'withdraw_velocity_factor':(.1,1.),
    'rotate_proximal_kp':(0,math.inf),'withdraw_proximal_kp':(0,math.inf),
    'rotate_proximal_kd':(0,math.inf),'withdraw_proximal_kd':(0,math.inf),
    'stage_waist_kp':(0,math.inf),'stage_waist_kd':(0,math.inf),
    'replay_waist_kp':(-1,math.inf),
    'demo_wrist_kp':(0,110),
    'demo_distal_wrist_kp':(0,220),
    'radial_offset_mm':(-100,100),'height_offset_mm':(-30,100),'tilt_deg':(0,60),'twist_deg':(0,120),
    'arc_center_toward_robot_mm':(0,30),
    'approach_center_height_mm':(100,200),
    'approach_tilt_deg':(5,45),'approach_twist_deg':(10,60),
    'path_tolerance_mm':(2,20),'target_tolerance_mm':(2,25),
    'position_search_fraction':(0,.9),'withdraw_height_mm':(60,200),'hand_back_yaw_deg':(5,45),
    'step_deg':(0,5),'arc_yaw_gain':(0,1),'max_index_tilt_deg':(10,50),'planning_margin_deg':(0,1),
    'left_sweep_hand_back_yaw_deg':(-20,20),'right_sweep_hand_back_yaw_deg':(-20,20),
    'sweep_hand_back_min_deg':(0,45),'sweep_hand_back_max_deg':(0,45),
    'arc_segments':(20,1000),'line_points':(2,1000),'max_attempts':(1,5000),'budget_seconds':(.1,90),
    'batch_trials':(1,5000),'batch_candidates':(1,32),'complete_candidates':(1,32),
    'max_repairs':(0,30),'backtrack_points':(1,100),
    'contact_branch_attempts':(1,32),'contact_search_seconds':(1,900),
    'contact_candidate_seconds':(1,900),
    'smoothing_joint_deviation_deg':(.1,5),
    'arm_velocity_rad_s':(.05,MEASURED_VELOCITY_LIMIT-TRACKING_VELOCITY_MARGIN),
    'waist_velocity_rad_s':(.05,MEASURED_VELOCITY_LIMIT-TRACKING_VELOCITY_MARGIN),
    'center_check_height_mm':(40,150),
    'endpoint_tolerance_deg':(.1,2.8),'endpoint_wait_seconds':(.1,30),
    'waist_kp':(20,80),'waist_kd':(.5,5),'waist_ki':(0,30),
    'waist_integral_torque_limit_nm':(.1,2),
    'waist_integral_recovery_error_deg':(math.degrees(.04),4.5),
    'waist_integral_build_rate_nm_s':(.1,1),'waist_integral_curve_scale':(.1,1),
    'arm_integral_recovery_error_deg':(math.degrees(.04),3.5),
    'arm_integral_hold_rate_nm_s':(.1,.5),
}
BOUNDS.update({side+'_'+j+'_'+gain:(0,(110 if j.startswith('wrist') else 100) if gain=='kp' else (8 if j=='wrist_roll' else 5))
               for side in ('left','right') for j in ARM_INTEGRAL_JOINTS for gain in ('kp','kd')})
BOUNDS.update({j+'_ki':(0,10) for j in ARM_INTEGRAL_JOINTS})
BOUNDS.update({'replay_'+j+'_kp':(-1,math.inf) for j in ARM_INTEGRAL_JOINTS})
BOUNDS.update({j+'_integral_limit_nm':(0,.3 if j.startswith('wrist') else 1.) for j in ARM_INTEGRAL_JOINTS})
BOUNDS['shoulder_pitch_integral_limit_nm']=(0,1.8)
BOUNDS['shoulder_roll_integral_limit_nm']=(0,3.6)


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader,node):
    result={}
    for key_node,value_node in node.value:
        key=loader.construct_object(key_node)
        if not isinstance(key,str) or key in result:raise ValueError('Duplicate or invalid configuration key: '+str(key))
        result[key]=loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,unique_mapping)


def load_parameters(path=DEFAULT_CONFIG):
    raw=Path(path).read_bytes()
    data=yaml.load(raw,Loader=UniqueLoader)
    if not isinstance(data,dict) or set(data)-set(SECTIONS):raise ValueError('Unknown planning configuration section')
    values={}
    for section,allowed in SECTIONS.items():
        supplied=data.get(section,{})
        if not isinstance(supplied,dict) or set(supplied)-set(allowed):raise ValueError('Unknown planning option in '+section)
        values.update(supplied)
    # Live files missing new timing switches must not silently restore stop/go
    # timing. Historical snapshots still use parameters_from_values unchanged.
    live_defaults=dict(smooth_contact_path=True,smooth_insert_path=True,smooth_joint_path=True)
    defaulted={key:value for key,value in live_defaults.items() if key not in values}
    parameters=parameters_from_values(dict(defaulted,**values))
    return parameters,dict(path=str(Path(path).resolve()),sha256=hashlib.sha256(raw).hexdigest(),
                           values=asdict(parameters),live_defaults_applied=defaulted)


def parameters_from_values(values):
    if not isinstance(values,dict) or set(values)-{f.name for f in fields(PlanningParameters)}:
        raise ValueError('Unknown planning configuration values')
    for field in fields(PlanningParameters):
        if field.name not in values:continue
        value=values[field.name]
        if field.type is bool:
            if type(value) is not bool:raise ValueError(field.name+' must be true or false')
            continue
        if (type(value) not in (int,float) or not math.isfinite(value) or
                (field.type is int and type(value) is not int)):
            raise ValueError(field.name+' has invalid numeric type')
        lo,hi=BOUNDS[field.name]
        if field.name in SECTIONS['demo_replay'] and value<0 and value!=-1:
            raise ValueError(field.name+' must be -1 (inherit) or nonnegative')
        if not lo<=value<=hi or (field.name=='step_deg' and value==0):
            raise ValueError(f'{field.name} outside [{lo},{hi}]')
        if hi==math.inf and value>STAGE_GAIN_FLOAT_MAX:
            raise ValueError(field.name+' cannot be represented as a finite LowCmd gain')
    parameters=PlanningParameters(**values)
    if parameters.replay_waist_kp not in (-1,0) and parameters.replay_waist_kp<20:
        raise ValueError('Replay waist Kp requires -1 (inherit), 0 (base gains), or Kp>=20')
    lo,hi=parameters.sweep_hand_back_min_deg,parameters.sweep_hand_back_max_deg
    if not parameters.hand_back_tangent_xy and (lo,hi)!=(0,0):
        margin=parameters.planning_margin_deg
        if not (lo<hi<=parameters.hand_back_yaw_deg and
                lo+margin < -parameters.left_sweep_hand_back_yaw_deg < hi-margin and
                lo+margin < parameters.right_sweep_hand_back_yaw_deg < hi-margin):
            raise ValueError('Hand-back yaw bounds require min < opposite-sweep preferences < max, '
                             'with planning margin; max must not exceed hand_back_yaw_deg')
    if (parameters.stage_waist_kp,parameters.stage_waist_kd)!=(0,0) and not (
            parameters.stage_waist_kp>=20 and parameters.stage_waist_kd>=.5):
        raise ValueError('Stage waist gains require both zero (base gains), or Kp>=20 and Kd>=0.5')
    return parameters


def configure_planner(planner,parameters):
    planner.planning_parameters=parameters
    planner.search_seconds=parameters.budget_seconds
    planner.search_attempts=parameters.max_attempts
    planner.max_index_tilt=parameters.max_index_tilt_deg


def rail_panel_parameters(parameters,record,overrides):
    """Freeze rail replay choices over an unchanged source-file snapshot."""
    if not isinstance(overrides,dict) or not overrides or set(overrides)-{
            'rail_replay_multiplier','rail_replay_fastest','rail_trim_idle_ends','rail_replay_cruise'}:
        raise ValueError('Invalid rail panel replay overrides')
    effective=parameters_from_values(dict(asdict(parameters),**overrides))
    return effective,dict(record,values=asdict(effective),rail_panel_base_values=dict(record['values']),
        rail_panel_overrides=dict(overrides))


def controller_bounded_parameters(parameters,record,gateway):
    """A planning-file edit may lower limits, but cannot raise the running gateway."""
    values={}
    for field,key in (('arm_velocity_rad_s','command_arm_velocity_limit_rad_s'),
                      ('waist_velocity_rad_s','command_waist_velocity_limit_rad_s')):
        if key not in gateway:continue
        limit=float(gateway[key])
        if not math.isfinite(limit) or limit<=0:raise ValueError('Invalid gateway command velocity limit')
        values[field]=min(getattr(parameters,field),limit)
    effective=replace(parameters,**values)
    record=dict(record,values=asdict(effective),requested_motion_limits={
        field:getattr(parameters,field) for field in SECTIONS['motion']})
    return effective,record
