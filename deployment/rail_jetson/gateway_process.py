"""Own only the gateway child created for an explicit initialize request."""
import signal
import subprocess


def silent(gateway):
    return (gateway.get('armed') == 'false'
            and gateway.get('command_publisher_exists') == 'false'
            and gateway.get('phase') in ('UNARMED', 'LATCHED_SILENT'))


class GatewayProcess:
    def __init__(self, root, config=None):
        self.root = root
        self.config = config or root / 'data/runtime_gateway.yaml'
        self.process = None
        self.stream = None

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        if self.running:
            return
        if self.stream:
            self.stream.close()
        logs = self.root / 'data/service_logs'
        logs.mkdir(parents=True, exist_ok=True)
        self.stream = (logs / 'gateway.log').open('a')
        command = [str(self.root / 'native_ws/install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'),
                   '--ros-args', '--params-file', str(self.config)]
        self.process = subprocess.Popen(command, stdout=self.stream, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def retire(self, gateway, fresh):
        if self.running:
            if not fresh or not silent(gateway):
                raise RuntimeError('Cannot exit gateway without fresh silent-output confirmation')
            self.process.send_signal(signal.SIGINT)
            # Do not force-kill an uncertain controller on timeout.
            self.process.wait(timeout=10.)
        self.process = None
        if self.stream:
            self.stream.close()
            self.stream = None
