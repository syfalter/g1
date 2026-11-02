"""Assumed constant-load sensitivity cases seeded from archived holding errors."""
import json
import argparse
import math
from pathlib import Path
import subprocess
import tempfile

from task_planning_config import load_parameters,parameters_from_values

ROOT=Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <iostream>
#include <iomanip>
int main() {
  double kp,kd,ki,limit,recovery,initial,load,inertia,rate,warm;int enabled;
  std::cout<<std::setprecision(12);
  while(std::cin>>kp>>kd>>ki>>limit>>recovery>>enabled>>initial>>load>>inertia>>rate>>warm) {
    g1_task::WaistIntegral controller;g1_task::WaistIntegralTuning tuning;
    tuning.recovery_error_rad=recovery*3.14159265358979323846/180.;
    double q=-initial,v=0,integral=0,peak_v=0,peak_i=0,peak_command=0;
    for(int tick=0;tick<3000 && std::abs(warm)>0;++tick)
      integral=controller.step(std::copysign(.02,warm),0,0,0,.01,enabled,false,ki,
                               std::min(limit,std::abs(warm)),rate,tuning).torque;
    for(int tick=0;tick<5000;++tick) {
      if(tick%10==0)integral=controller.step(-q,0,0,v,.01,enabled,false,ki,limit,
        g1_task::arm_integral_rate_scale(0,0,v,rate),tuning).torque;
      const double command=-kp*q-kd*v+integral;
      v+=(command-load)/inertia*.001;q+=v*.001;
      peak_v=std::max(peak_v,std::abs(v));peak_i=std::max(peak_i,std::abs(integral));
      peak_command=std::max(peak_command,std::abs(command));
    }
    std::cout<<std::abs(q)<<" "<<peak_v<<" "<<peak_i<<" "<<peak_command<<"\n";
  }
}
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=ROOT/'logs/tracking_tuning_20260911')
    parser.add_argument('--old-plan',type=Path,default=ROOT/'data/task_sessions/20260911_221536_397866/plan_0026/plan.json')
    parser.add_argument('--trace-prefix',default='endpoint_motion_20260911_22')
    parser.add_argument('--carried-integral',action='store_true')
    parser.add_argument('--output-name',default='plant.json')
    args=parser.parse_args();directory=args.directory
    history=json.loads((directory/'history.json').read_text())
    archive=args.old_plan
    old_record=json.loads(archive.read_text())['planning_config']
    old=parameters_from_values(old_record['values']);new,config=load_parameters()
    cases=[]
    for trace in history['traces']:
        if not Path(trace['path']).name.startswith(args.trace_prefix):continue
        for joint,r in trace['joints'].items():
            if joint=='waist_yaw_joint' or r['steady_samples']<20:continue
            kp=getattr(old,joint[:-6]+'_kp')
            cases.append(dict(source=trace['path'],joint=joint,initial_error=r['steady_signed_error_rad'],
                initial_integral=r.get('steady_integral_nm') or 0.,
                disturbance=kp*r['steady_signed_error_rad']+(r.get('steady_integral_nm') or 0.)))
    if not args.carried_integral and (archive.parent/'execution_failure.json').is_file():
        failure=json.loads((archive.parent/'execution_failure.json').read_text())
        joint='right_shoulder_pitch_joint'
        error=failure['expected_endpoint'][joint]-failure['feedback_at_failure']['joints'][joint]
        cases.append(dict(source=str(archive.parent/'execution_failure.json'),joint=joint,
                          initial_error=error,initial_integral=0.,disturbance=old.right_shoulder_pitch_kp*error))
    inputs=[];records=[]
    for case in cases:
        side,suffix=case['joint'].split('_',1);suffix=suffix[:-6]
        for inertia in (.03,.1,.4,1.):
            for load_scale in (.9,1.,1.1):
                for sign in (-1.,1.):
                    for name,params in (('old',old),('new',new)):
                        enabled=getattr(params,side+'_arm_integral_enabled') and (side=='left' or params.passive_arm_integral_enabled)
                        values=[getattr(params,side+'_'+suffix+'_kp'),getattr(params,side+'_'+suffix+'_kd'),
                                getattr(params,suffix+'_ki'),getattr(params,suffix+'_integral_limit_nm'),
                                params.arm_integral_recovery_error_deg,int(enabled),case['initial_error']*sign,
                                case['disturbance']*load_scale*sign,inertia,params.arm_integral_hold_rate_nm_s,
                                case['initial_integral']*sign if args.carried_integral else 0.]
                        inputs.append(' '.join(map(str,values)))
                        records.append(dict(**case,version=name,inertia=inertia,load_scale=load_scale,sign=sign,
                                            integral_limit_nm=values[3]))
    with tempfile.TemporaryDirectory() as temp:
        binary=Path(temp)/'plant'
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-x','c++','-',
            '-I'+str(ROOT/'src/g1_unitree_control/include'),'-o',str(binary)],
            input=SOURCE,text=True,check=True,capture_output=True)
        result=subprocess.run([str(binary)],input='\n'.join(inputs),text=True,check=True,capture_output=True)
    outputs=result.stdout.splitlines();assert len(outputs)==len(records)
    for record,line in zip(records,outputs):
        values=list(map(float,line.split()));assert all(math.isfinite(v) for v in values)
        record.update(zip(('final_error_rad','peak_velocity_rad_s','peak_integral_nm','peak_pd_plus_i_nm'),values))
        assert record['peak_integral_nm']<=record['integral_limit_nm']+1e-9
    summary=[]
    for joint in sorted({r['joint'] for r in records}):
        for version in ('old','new'):
            rows=[r for r in records if r['joint']==joint and r['version']==version]
            nominal=[r for r in rows if r['load_scale']==1.]
            summary.append(dict(joint=joint,version=version,cases=len(rows),
                worst_nominal_final_deg=max(math.degrees(r['final_error_rad']) for r in nominal),
                worst_sensitivity_final_deg=max(math.degrees(r['final_error_rad']) for r in rows),
                peak_velocity_rad_s=max(r['peak_velocity_rad_s'] for r in rows)))
    report=dict(configuration=config,old_configuration=old_record,summary=summary,cases=records,
        hardware_commands_sent=0,ros_initialized=False,
        limitations=['Constant-load single-axis model; inertia and +/-10% load are assumptions, not identification.',
                     'Integral initialized from archived median torque.' if args.carried_integral else
                     'All cases reset integral initially, unlike an uninterrupted real task.',
                     'No contact, coupling, friction or motor delay model. No claim of physical smoothness.',
                     'PD+I omits gravity/feedforward. Some sensitivity cases can still exceed endpoint tolerance.'])
    (directory/args.output_name).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
