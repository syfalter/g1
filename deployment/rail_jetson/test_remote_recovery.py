import unittest

from remote_recovery import RecoveryChord


class ChordTests(unittest.TestCase):
    def test_startup_held_is_not_confirmation(self):
        chord = RecoveryChord()
        chord.observe(0x110, 0.)
        chord.observe(0x110, 1.)
        self.assertFalse(chord.pending(1.))
        chord.observe(0, 2.)
        chord.observe(0x110, 3.)
        self.assertTrue(chord.pending(3.))

    def test_stop_priority_and_no_reuse(self):
        chord = RecoveryChord()
        chord.observe(0, 0.)
        chord.observe(0x110, 1.)
        chord.observe(0x330, 2.)
        chord.observe(0x110, 3.)
        self.assertFalse(chord.pending(3.))
        chord.observe(0, 4.)
        chord.observe(0x110, 5.)
        self.assertTrue(chord.pending(5.))
        chord.cancel('consumed')
        chord.observe(0x110, 6.)
        self.assertFalse(chord.pending(6.))

    def test_expiry_does_not_extend_on_hold(self):
        chord = RecoveryChord()
        chord.observe(0, 0.)
        chord.observe(0x110, 1.)
        chord.observe(0x110, 30.)
        self.assertFalse(chord.pending(31.))
