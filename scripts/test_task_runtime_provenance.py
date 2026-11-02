"""Native rebuilds must invalidate a frozen replay, without mutating its snapshot."""
from pathlib import Path
import tempfile
import unittest

from task_runtime_provenance import RUNTIME_FILES, runtime_hashes, runtime_matches, snapshot_runtime


class RuntimeProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'workspace'
        for name in RUNTIME_FILES:
            path = self.root/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(name)

    def test_source_and_binary_changes_invalidate(self):
        original = runtime_hashes(self.root)
        self.assertTrue(runtime_matches(original, self.root))
        for name in RUNTIME_FILES:
            path = self.root/name
            path.write_text('modified')
            self.assertFalse(runtime_matches(original, self.root), name)
            path.write_text(name)
        self.assertTrue(runtime_matches(original, self.root))

    def test_missing_or_incomplete_identity_rejected(self):
        original = runtime_hashes(self.root)
        incomplete = dict(original)
        incomplete.pop(RUNTIME_FILES[0])
        self.assertFalse(runtime_matches(incomplete, self.root))
        (self.root/RUNTIME_FILES[0]).unlink()
        self.assertFalse(runtime_matches(original, self.root))

    def test_snapshot_copies_symlink_target_and_survives_rebuild(self):
        path = self.root/RUNTIME_FILES[6]
        binary = self.root/'built-binary'
        path.rename(binary)
        path.symlink_to(binary)
        snapshot = Path(self.temp.name)/'snapshot'
        original = snapshot_runtime(snapshot, self.root)
        self.assertFalse((snapshot/RUNTIME_FILES[6]).is_symlink())
        binary.write_text('rebuilt')
        self.assertFalse(runtime_matches(original, self.root))
        self.assertEqual(runtime_hashes(snapshot), original)
        with self.assertRaises(FileExistsError):
            snapshot_runtime(snapshot, self.root)


if __name__ == '__main__':
    unittest.main()
