"""Exact C++ policy replay and explicitly simplified single-axis plant checks."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import tempfile

from task_planning_config import load_parameters,parameters_from_values

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include "g1_unitree_control/task_tracking_scale.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 10) return 2;
  const std::string mode=argv[1];
  const double kp=std::stod(argv[2]),kd=std::stod(argv[3]),ki=std::stod(argv[4]);
  const double limit=std::stod(argv[5]);
  g1_task::WaistIntegralTuning tuning;
  tuning.recovery_error_rad=std::stod(argv[6])*3.14159265358979323846/180.;
  tuning.build_rate_nm_s=std::stod(argv[7]); tuning.curve_gain_scale=std::stod(argv[8]);
  tuning.smooth_stiction=std::stoi(argv[9])!=0;
  std::cout<<std::setprecision(12);
  if (mode=="replay") {
    g1_task::WaistIntegral controller;
    double error,dv,da,v; int active;
    while (std::cin>>error>>dv>>da>>v>>active) {
      const auto r=controller.step(error,dv,da,v,.01,true,active!=0,ki,limit,1.,tuning);
      std::cout<<r.torque<<" "<<kp*error+kd*(dv-v)+r.torque<<" "
               <<g1_task::waist_integral_mode_name(r.mode)<<"\n";
    }
    return 0;
  }
  if (mode!="plant") return 3;
  // These inertias and friction ranges are sensitivity assumptions, not identified motors.
  for (int kind : {0,1,2}) for (double inertia : {.08,.2,.6,1.5})
    for (double multiplier : {.9,1.,1.1}) for (double sign : {-1.,1.}) {
      const double load=60.*.05215*multiplier;
      double q=kind==2 ? 0. : -sign*load/60.,v=0.,integral=0.,desired=0.,dv=0.;
      double progress=0.,scale=1.,last_scale=1.,peak_v=0.,peak_torque=0.,peak_i=0.;
      double min_scale=1.,complete=-1.,peak_error=0.,max_i_delta=0.;
      const double duration=12.344768547,dt=.001;
      int reversals=0,previous_direction=0;
      g1_task::WaistIntegral controller;
      for (int tick=0;tick<(kind==2 ? 40000 : 5000);++tick) {
        if (tick%10==0) {
          const bool active=kind==2 && progress<duration;
          double raw_dv=0.,raw_da=0.;
          if (kind==2) {
            const double u=std::min(1.,progress/duration),delta=sign*.1363;
            desired=delta*(10*u*u*u-15*u*u*u*u+6*u*u*u*u*u);
            raw_dv=delta*(30*u*u-60*u*u*u+30*u*u*u*u)/duration;
            raw_da=delta*(60*u-180*u*u+120*u*u*u)/(duration*duration);
          }
          const double error=desired-q;
          scale=active ? g1_task::tracking_time_scale(error,.06,.10) : 0.;
          const double scale_rate=(scale-last_scale)/.01;
          dv=raw_dv*scale;
          const double da=active ? raw_da*scale*scale+raw_dv*scale_rate : 0.;
          const auto r=controller.step(error,dv,da,v,.01,true,active,ki,limit,1.,tuning);
          max_i_delta=std::max(max_i_delta,std::abs(r.torque-integral));
          integral=r.torque;peak_i=std::max(peak_i,std::abs(integral));
          if (active) {
            min_scale=std::min(min_scale,scale);
            progress+=.01*scale;
            if (progress>=duration && complete<0.) complete=tick*dt;
          }
          last_scale=scale;
        }
        const double torque=kp*(desired-q)+kd*(dv-v)+integral;
        peak_torque=std::max(peak_torque,std::abs(torque));
        if (kind==0) {
          v+=(torque-sign*load)/inertia*dt;
        } else {
          // Stiction holds until static friction is exceeded; sliding friction is smaller.
          if (std::abs(v)<.0001 && std::abs(torque)<=load) v=0.;
          else {
            const double direction=std::abs(v)>.0001 ? std::copysign(1.,v) : std::copysign(1.,torque);
            const double next=v+(torque-direction*.78*load-.3*v)/inertia*dt;
            v=(v*next<0 && std::abs(torque)<=load) ? 0. : next;
          }
        }
        q+=v*dt;peak_v=std::max(peak_v,std::abs(v));
        peak_error=std::max(peak_error,std::abs(desired-q));
        const int direction=v>.005 ? 1 : v<-.005 ? -1 : 0;
        if (direction && previous_direction && direction!=previous_direction) ++reversals;
        if (direction) previous_direction=direction;
        if (kind==2 && complete>=0. && tick*dt>=complete+5.) break;
      }
      std::cout<<kind<<" "<<inertia<<" "<<multiplier<<" "<<sign<<" "<<std::abs(desired-q)
               <<" "<<peak_v<<" "<<peak_torque<<" "<<peak_i<<" "<<min_scale<<" "<<complete
               <<" "<<reversals<<" "<<max_i_delta<<" "<<peak_error<<"\n";
    }
}
'''


def compile_probe(directory):
    binary = Path(directory) / 'waist_probe'
    subprocess.run(['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-x', 'c++', '-',
                    '-I' + str(ROOT / 'src/g1_unitree_control/include'), '-o', str(binary)],
                   input=SOURCE, text=True, check=True, capture_output=True)
    return binary


def settings(parameters):
    return [parameters.waist_kp, parameters.waist_kd, parameters.waist_ki,
            parameters.waist_integral_torque_limit_nm, parameters.waist_integral_recovery_error_deg,
            parameters.waist_integral_build_rate_nm_s, parameters.waist_integral_curve_scale,
            int(parameters.waist_integral_smooth_stiction)]


def run_probe(binary, mode, values, text=''):
    result = subprocess.run([str(binary), mode, *map(str, values)], input=text,
                            text=True, check=True, capture_output=True)
    return [row.split() for row in result.stdout.splitlines()]


def replay(binary, path, values):
    with path.open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['event'] in ('sample', 'settling_hold')]
    lines = []
    for r in rows:
        error = float(r['waist_yaw_joint_target']) - float(r['waist_yaw_joint_q'])
        lines.append(' '.join(map(str, [error, r['waist_yaw_joint_command_dq'],
            r['waist_yaw_joint_command_ddq'], r['waist_yaw_joint_dq'], int(r['event'] == 'sample')])))
    outputs = run_probe(binary, 'replay', values, '\n'.join(lines))
    assert len(outputs) == len(rows)
    torque = [float(r[0]) for r in outputs]
    hold = [i for i, r in enumerate(rows) if r['event'] == 'settling_hold']
    from collections import Counter
    return dict(path=str(path), rows=len(rows), initial_integral_nm=0.,
                peak_integral_nm=max(map(abs, torque)),
                peak_step_nm=max(abs(b-a) for a, b in zip([0.]+torque, torque)),
                peak_pd_plus_integral_nm=max(abs(float(r[1])) for r in outputs),
                final_integral_nm=torque[-1], modes=dict(Counter(r[2] for r in outputs)),
                hold_modes=dict(Counter(outputs[i][2] for i in hold)),
                hold_final_integral_nm=torque[hold[-1]] if hold else None)


def plant(binary, values):
    names = ('kind', 'inertia', 'load_multiplier', 'sign', 'final_error_rad', 'peak_velocity_rad_s',
             'peak_pd_plus_integral_nm', 'peak_integral_nm', 'min_time_scale', 'completion_s',
             'velocity_reversals', 'max_integral_step_nm', 'peak_error_rad')
    return [dict(zip(names, map(float, row))) for row in run_probe(binary, 'plant', values)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'logs/waist_recovery_20260911/replay.json')
    parser.add_argument('--old-plan',type=Path)
    parser.add_argument('--trace-glob',default='endpoint_motion_20260911_214*.csv')
    args = parser.parse_args()
    parameters, config = load_parameters()
    old = [60., 4.5, 20., 1.2, .04*180/3.141592653589793, 1., .25, 0]
    if args.old_plan:
        old=settings(parameters_from_values(json.loads(args.old_plan.read_text())['planning_config']['values']))
    new = settings(parameters)
    logs = sorted((ROOT/'logs/endpoint_motion').glob(args.trace_glob))
    with tempfile.TemporaryDirectory() as directory:
        binary = compile_probe(directory)
        report = dict(hardware_commands_sent=0, ros_initialized=False, configuration=config,
                      old_settings=old, new_settings=new,
                      fixed_trace_replay=[dict(old=replay(binary, p, old), new=replay(binary, p, new)) for p in logs],
                      simplified_plant=dict(old=plant(binary, old), new=plant(binary, new)),
                      limitations=['Replay keeps old measured joints fixed and does not predict new closed-loop tracking.',
                                   'Plant inertia/friction are assumed; no robot/contact dynamics identification.',
                                   'No physical smoothness or full D-ring success is certified by these checks.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(args.output)
    for version in ('old', 'new'):
        rows = report['simplified_plant'][version]
        print(version, '5s nominal-load hold errors (rad):',
              [round(r['final_error_rad'], 5) for r in rows if r['kind']==0 and r['load_multiplier']==1. and r['sign']==1.])
        print(version, 'plant peak speed:', max(r['peak_velocity_rad_s'] for r in rows))


if __name__ == '__main__':
    main()
