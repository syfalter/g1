"""Counterfactual replay of the exact waist integrator on recorded motion CSVs."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import tempfile


SOURCE=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> result;
  std::string field;
  bool quoted=false;
  for (std::size_t i=0;i<line.size();++i) {
    const char c=line[i];
    if (c=='"') {
      if (quoted && i+1<line.size() && line[i+1]=='"') { field.push_back('"'); ++i; }
      else quoted=!quoted;
    } else if (c==',' && !quoted) {
      result.push_back(field); field.clear();
    } else field.push_back(c);
  }
  result.push_back(field);
  return result;
}

int main(int argc,char** argv) {
  if (argc!=4) return 2;
  const double ki=std::stod(argv[2]),limit=std::stod(argv[3]);
  std::ifstream input(argv[1]);
  std::string line;
  if (!std::getline(input,line)) throw std::runtime_error("empty CSV");
  const auto header=split_csv(line);
  std::map<std::string,std::size_t> column;
  for (std::size_t i=0;i<header.size();++i) column[header[i]]=i;
  const auto index=[&](const std::string& name) {
    const auto found=column.find(name);
    if (found==column.end()) throw std::runtime_error("missing column: "+name);
    return found->second;
  };
  const auto time_i=index("ros_time_s");
  const auto scale_i=index("time_scale");
  const auto event_i=index("event");
  const auto target_i=index("waist_yaw_joint_target");
  const auto ddq_i=index("waist_yaw_joint_target_ddq");
  const auto command_dq_i=index("waist_yaw_joint_command_dq");
  const auto q_i=index("waist_yaw_joint_q");
  const auto dq_i=index("waist_yaw_joint_dq");
  const auto tau_i=index("waist_yaw_joint_tau_est");

  g1_task::WaistIntegral controller;
  std::map<std::string,std::size_t> modes;
  double previous_time=0,previous_integral=0;
  double peak_integral=0,peak_cycle_delta=0,peak_pd=0,peak_pid=0;
  double peak_measured_velocity=0,peak_tau_est=0,final_error=0,final_integral=0;
  double integral_at_first_breakaway=0;
  bool have_time=false,have_breakaway=false;
  std::size_t rows=0,saturated_rows=0;
  while (std::getline(input,line)) {
    const auto fields=split_csv(line);
    if (fields.size()!=header.size()) throw std::runtime_error("malformed CSV row");
    const double stamp=std::stod(fields[time_i]);
    const double time_scale=std::stod(fields[scale_i]);
    const double target=std::stod(fields[target_i]);
    const double command_dq=std::stod(fields[command_dq_i]);
    const double target_ddq=std::stod(fields[ddq_i])*time_scale*time_scale;
    const double measured=std::stod(fields[q_i]);
    const double measured_dq=std::stod(fields[dq_i]);
    const double tau_est=std::stod(fields[tau_i]);
    double dt=have_time ? stamp-previous_time : .01;
    if (!(dt>0 && dt<=.05)) dt=.01;
    previous_time=stamp;have_time=true;
    const double error=target-measured;
    const bool moving=fields[event_i]=="sample";
    const auto result=controller.step(error,command_dq,target_ddq,measured_dq,dt,
                                      true,moving,ki,limit);
    const std::string mode=g1_task::waist_integral_mode_name(result.mode);
    ++modes[mode];
    if (result.saturated) ++saturated_rows;
    peak_integral=std::max(peak_integral,std::abs(result.torque));
    peak_cycle_delta=std::max(peak_cycle_delta,std::abs(result.torque-previous_integral));
    previous_integral=result.torque;
    const double pd=60.0*error-4.5*(measured_dq-command_dq);
    peak_pd=std::max(peak_pd,std::abs(pd));
    peak_pid=std::max(peak_pid,std::abs(pd+result.torque));
    peak_measured_velocity=std::max(peak_measured_velocity,std::abs(measured_dq));
    peak_tau_est=std::max(peak_tau_est,std::abs(tau_est));
    if (!have_breakaway && std::abs(measured_dq)>.08) {
      have_breakaway=true;integral_at_first_breakaway=result.torque;
    }
    final_error=error;final_integral=result.torque;++rows;
  }
  auto hold_controller=controller;
  double projected_hold_integral=final_integral;
  double projected_limit_time=-1.0;
  for (int i=1;i<=500;++i) {
    const auto result=hold_controller.step(
      final_error,0,0,0,.01,true,false,ki,limit);
    projected_hold_integral=result.torque;
    if (projected_limit_time<0 && result.saturated) projected_limit_time=i*.01;
  }
  std::cout<<std::fixed<<std::setprecision(9)<<"{\n"
    <<"  \"path\": \""<<argv[1]<<"\",\n"
    <<"  \"rows\": "<<rows<<",\n"
    <<"  \"peak_integral_torque_nm\": "<<peak_integral<<",\n"
    <<"  \"peak_integral_cycle_delta_nm\": "<<peak_cycle_delta<<",\n"
    <<"  \"integral_at_first_measured_dq_above_0_08_nm\": "
    <<(have_breakaway ? integral_at_first_breakaway : 0.0)<<",\n"
    <<"  \"breakaway_observed\": "<<(have_breakaway ? "true" : "false")<<",\n"
    <<"  \"peak_new_pd_only_command_nm\": "<<peak_pd<<",\n"
    <<"  \"peak_new_pid_command_nm\": "<<peak_pid<<",\n"
    <<"  \"peak_recorded_measured_velocity_rad_s\": "<<peak_measured_velocity<<",\n"
    <<"  \"peak_recorded_tau_est_nm\": "<<peak_tau_est<<",\n"
    <<"  \"final_recorded_tracking_error_rad\": "<<final_error<<",\n"
    <<"  \"final_counterfactual_integral_nm\": "<<final_integral<<",\n"
    <<"  \"projected_integral_after_5s_frozen_hold_nm\": "<<projected_hold_integral<<",\n"
    <<"  \"projected_pid_after_5s_frozen_hold_nm\": "
    <<60.0*final_error+projected_hold_integral<<",\n"
    <<"  \"projected_time_to_integral_limit_s\": "<<projected_limit_time<<",\n"
    <<"  \"integral_saturated_rows\": "<<saturated_rows<<",\n"
    <<"  \"mode_rows\": {";
  bool first=true;
  for (const auto& item:modes) {
    if (!first) std::cout<<',';
    std::cout<<"\n    \""<<item.first<<"\": "<<item.second;first=false;
  }
  if (!modes.empty()) std::cout<<'\n';
  std::cout<<"  }\n}\n";
}
'''


def main():
    root=Path(__file__).resolve().parents[1]
    defaults=[
        root/'logs/endpoint_motion/endpoint_motion_20260910_131746_001.csv',
        root/'logs/endpoint_motion/endpoint_motion_20260910_132222_002.csv',
        root/'logs/endpoint_motion/endpoint_motion_20260910_132406_003.csv']
    parser=argparse.ArgumentParser()
    parser.add_argument('csv',nargs='*',type=Path,default=defaults)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--ki',type=float,default=20.)
    parser.add_argument('--integral-limit',type=float,default=1.2)
    parser.add_argument('--include-dir',type=Path,default=root/'src/g1_unitree_control/include')
    args=parser.parse_args()
    output=args.output or root/'logs/waist_integral_replay'/datetime.now().strftime('%Y%m%d_%H%M%S')/'report.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as directory:
        binary=Path(directory)/'waist_integral_replay'
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-x','c++','-',
                        '-I'+str(args.include_dir.resolve()),'-o',str(binary)],
                       input=SOURCE,text=True,check=True,capture_output=True)
        results=[]
        for path in args.csv:
            result=subprocess.run([str(binary),str(path.resolve()),str(args.ki),str(args.integral_limit)],text=True,check=True,
                                  capture_output=True)
            results.append(json.loads(result.stdout))
    report={'counterfactual_only':True,'hardware_commands_sent':0,
            'initial_integral_nm':0.,'measurement_trace_fixed':True,
            'include_dir':str(args.include_dir.resolve()),
            'controller':{'kp':60.0,'kd':4.5,'ki':args.ki,'integral_limit_nm':args.integral_limit},
            'inputs':[str(path.resolve()) for path in args.csv],'results':results}
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(output)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
