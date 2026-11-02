"""Read recorded joint data and replay the exact C++ integral; no ROS or hardware access."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import tempfile

from task_planning_config import ARM_INTEGRAL_JOINTS,DEFAULT_CONFIG,load_parameters

SOURCE=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <array>
#include <iomanip>
#include <iostream>
int main() {
  std::array<g1_task::WaistIntegral,7> controllers;
  int joint; double dt,error,dq,ddq,measured_dq,ki,limit;
  std::cout<<std::setprecision(12);
  while (std::cin>>joint>>dt>>error>>dq>>ddq>>measured_dq>>ki>>limit) {
    auto r=controllers.at(joint).step(error,dq,ddq,measured_dq,dt,true,true,ki,limit,.25);
    std::cout<<r.torque<<' '<<r.saturated<<' '
             <<g1_task::waist_integral_mode_name(r.mode)<<'\n';
  }
}
'''


def replay(path, parameters, binary):
    with path.open() as stream:rows=list(csv.DictReader(stream))
    names=['left_'+joint+'_joint' for joint in ARM_INTEGRAL_JOINTS]
    inputs=[];samples=[];previous=None
    for row in rows:
        stamp=float(row['ros_time_s'])
        dt=.01 if previous is None else stamp-previous
        if dt<=0:continue
        if dt>.05:raise ValueError('Trace gap exceeds 50ms; do not fabricate missing feedback')
        previous=stamp
        for i,(joint,name) in enumerate(zip(ARM_INTEGRAL_JOINTS,names)):
            error=float(row[name+'_target'])-float(row[name+'_q'])
            values=[dt,error,float(row[name+'_command_dq']),float(row[name+'_command_ddq']),
                    float(row[name+'_dq']),getattr(parameters,joint+'_ki'),
                    getattr(parameters,joint+'_integral_limit_nm')]
            if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite trace')
            inputs.append(' '.join(map(str,[i,*values])))
            samples.append((i,error,dt))
    output=subprocess.run([str(binary)],input='\n'.join(inputs)+'\n',text=True,
                          capture_output=True,check=True).stdout.splitlines()
    if len(output)!=len(samples):raise ValueError('Incomplete C++ replay output')
    summaries=[dict(joint=name,peak_integral_nm=0.,final_integral_nm=0.,saturated_samples=0,
                    peak_rate_nm_s=0.,modes={}) for name in names]
    previous_torque=[0.]*7
    for (i,error,dt),line in zip(samples,output):
        torque,saturated,mode=line.split();torque=float(torque);record=summaries[i]
        record['peak_integral_nm']=max(record['peak_integral_nm'],abs(torque))
        record['peak_rate_nm_s']=max(record['peak_rate_nm_s'],abs(torque-previous_torque[i])/dt)
        record['final_integral_nm']=torque
        record['final_recorded_error_rad']=error
        record['saturated_samples']+=int(saturated)
        record['modes'][mode]=record['modes'].get(mode,0)+1
        limit=getattr(parameters,ARM_INTEGRAL_JOINTS[i]+'_integral_limit_nm')
        assert abs(torque)<=limit+1e-9
        assert abs(torque-previous_torque[i])<=.75*dt+1e-8
        previous_torque[i]=torque
    return dict(path=str(path.resolve()),recorded_rows=len(rows),joints=summaries)


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser()
    parser.add_argument('csv',type=Path,nargs='+')
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();parameters,record=load_parameters(args.config)
    with tempfile.TemporaryDirectory() as directory:
        binary=Path(directory)/'replay'
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-x','c++','-',
                        '-I'+str(root/'src/g1_unitree_control/include'),'-o',str(binary)],
                       input=SOURCE,text=True,check=True,capture_output=True)
        results=[replay(path,parameters,binary) for path in args.csv]
    report=dict(counterfactual_only=True,measurement_trace_fixed=True,initial_integral_nm=0.,
        hardware_commands_sent=0,planning_config=record,results=results,
        limitation='Fixed historical readings cannot establish closed-loop improvement or contact safety.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(args.output)
    for result in results:
        print(result['path'])
        for joint in result['joints']:
            print(joint['joint'],'peak_I',round(joint['peak_integral_nm'],4),
                  'final_I',round(joint['final_integral_nm'],4),'saturated',joint['saturated_samples'])


if __name__=='__main__':main()
