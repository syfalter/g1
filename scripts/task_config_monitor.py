"""Read-only config monitoring. Never applies gains or sends robot commands."""
from pathlib import Path
import time

from task_planning_config import SECTIONS, load_parameters, controller_bounded_parameters, rail_panel_parameters


class PlanningConfigMonitor:
    def __init__(self,path,clock=time.monotonic):
        self.path=Path(path)
        self.clock=clock
        self.parameters=None
        self.record=None
        self.startup=None
        self.error=None
        self.signature=None
        self.candidate=None
        self.candidate_since=0.
        self.next_check=0.
        self.revision=0
        self.reload()

    def reload(self):
        previous=(self.record['values'] if self.record else None,self.error)
        try:
            parameters,record=load_parameters(self.path)
            self.parameters,self.record=parameters,record
            if self.startup is None:self.startup=dict(record['values'])
            self.error=None
        except (OSError,ValueError,TypeError) as error:
            self.error=str(error)
        except Exception as error:
            # YAML parser exceptions must not terminate the panel refresh loop.
            self.error=str(error)
        current=(self.record['values'] if self.record else None,self.error)
        changed=previous!=current
        if changed:self.revision+=1
        return changed

    def poll(self):
        now=self.clock()
        if now<self.next_check:return False
        self.next_check=now+.5
        try:
            stat=self.path.stat();signature=(stat.st_mtime_ns,stat.st_size,stat.st_ino)
        except OSError:signature=('missing',)
        if signature!=self.candidate:
            self.candidate=signature;self.candidate_since=now
            return False
        if now-self.candidate_since<.5 or signature==self.signature:return False
        self.signature=signature
        return self.reload()

    def matches(self,record,gateway):
        if self.error or self.record is None:return False
        parameters,effective=controller_bounded_parameters(self.parameters,self.record,gateway)
        if 'rail_panel_overrides' in record or 'rail_panel_base_values' in record:
            if record.get('rail_panel_base_values')!=effective['values']:return False
            try:_,effective=rail_panel_parameters(parameters,effective,record.get('rail_panel_overrides'))
            except (ValueError,TypeError):return False
        return record.get('values')==effective['values']

    def restart_fields(self,gateway=None):
        if self.record is None or self.startup is None:return []
        values=self.record['values']
        fields=[k for section in ('arm_pd','arm_integral','waist_controller')
                for k in SECTIONS[section] if values[k]!=self.startup[k]]
        fields.extend(k for k in SECTIONS['motion'] if values[k]>self.startup[k])
        gateway=gateway or {}
        if (gateway.get('task_stage_profile','normal')=='normal' and
            gateway.get('task_stage_switching','false')=='false' and
            gateway.get('phase') in ('ARMED_HOLD','UNARMED')):
            for key in SECTIONS['arm_pd']:
                diagnostic=key[:-3]+'_joint_'+key[-2:]
                if diagnostic not in gateway:continue
                if abs(float(gateway[diagnostic])-float(values[key]))>1e-5:
                    if key not in fields:fields.append(key)
                elif key in fields:fields.remove(key)
        for key in ('waist_kp','waist_kd','waist_ki','waist_integral_torque_limit_nm'):
            if key not in gateway:continue
            if abs(float(gateway[key])-float(values[key]))>1e-5:
                if key not in fields:fields.append(key)
            elif key in fields:fields.remove(key)
        return fields

    def execution_issue(self,gateway=None):
        if self.error:return 'task_planning.yaml 无效：'+self.error
        fields=self.restart_fields(gateway)
        if fields:return '控制启动参数尚未应用，需要重启控制器：'+', '.join(fields)
        return None

    def summary(self,gateway=None):
        if self.error:return '配置无效：'+self.error
        fields=self.restart_fields(gateway)
        digest=self.record['sha256'][:8] if self.record else '-'
        return '配置 '+digest+' | '+(f'{len(fields)} 项启动参数未应用，需重启' if fields else '已读取；规划及阶段参数用于新计划')
