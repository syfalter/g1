"""Fresh physical recovery intent; startup-held keys never count as a press."""


class RecoveryChord:
    def __init__(self):
        self.held = None
        self.deadline = 0.
        self.status = 'waiting for new R2+A'

    def observe(self, keys, now):
        held = keys & 0x0110 == 0x0110
        if keys & 0x0220 == 0x0220:
            self.cancel('L2+B has priority')
        elif held and self.held is False:
            self.deadline = now + 30.
            self.status = 'R2+A received; waiting for stationary healthy feedback'
        self.held = held

    def pending(self, now):
        if self.deadline and now >= self.deadline:
            self.cancel('R2+A expired; release and press again')
        return bool(self.deadline)

    def cancel(self, reason):
        self.deadline = 0.
        self.status = reason
