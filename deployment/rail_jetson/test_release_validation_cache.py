import copy
import unittest
from unittest.mock import patch

from bundle import digest
from executor import alignment_already_satisfied
from release_validation_cache import ReleaseValidationCache, PinnedRelease
from releases import validate
from test_cloud_executor import release


class ValidationTests(unittest.TestCase):
    def test_pinned_version_has_no_per_request_copy_or_hash(self):
        source = release()
        pinned = PinnedRelease(source)
        cache = ReleaseValidationCache()
        with patch('release_validation_cache.digest', side_effect=AssertionError('per-request hash')):
            with patch('release_validation_cache.copy.deepcopy', side_effect=AssertionError('per-request copy')):
                self.assertIs(cache.snapshot(pinned), cache.snapshot(pinned))
        source['payload']['version'] = 'outside-edit'
        self.assertNotEqual(pinned.snapshot['payload']['version'], 'outside-edit')

    def test_pinned_nested_mutation_fails_but_transport_copy_is_mutable(self):
        pinned = PinnedRelease(release())
        data = pinned.snapshot
        with self.assertRaises(TypeError):
            data['payload']['version'] = 'changed'
        with self.assertRaises(TypeError):
            data['payload'].update(version='changed')
        action = next(iter(data['payload']['actions'].values()))
        positions = next(s['trajectory']['points'][0]['positions'] for s in action['program'] if s['kind']=='recorded')
        with self.assertRaises(TypeError):
            positions[0] = .5
        with self.assertRaises(TypeError):
            positions.append(.5)
        with self.assertRaises(TypeError):
            pinned._snapshot = {}
        self.assertEqual(digest(data['payload']), data['sha256'])
        copied = copy.deepcopy(data)
        self.assertIs(type(copied), dict)
        self.assertEqual(copied, data)
        copied['payload']['version'] = 'private-copy'
        self.assertNotEqual(copied, data)

    def test_validate_once_but_hash_every_request(self):
        source = release()
        cache = ReleaseValidationCache()
        with patch('release_validation_cache.validate', wraps=validate) as checked:
            first = cache.snapshot(source)
            second = cache.snapshot(source)
            self.assertEqual(checked.call_count, 1)
            self.assertIsNot(first, second)
            source['payload']['version'] = 'tampered'
            with self.assertRaises(ValueError):
                cache.snapshot(source)
            self.assertEqual(checked.call_count, 1)

    def test_new_content_revalidates_even_with_new_hash(self):
        source = release()
        cache = ReleaseValidationCache()
        cache.snapshot(source)
        source['payload']['initialization']['velocity_scale'] = 1.
        source['sha256'] = digest(source['payload'])
        with self.assertRaises(ValueError):
            cache.snapshot(source)

    def test_private_snapshots_and_valid_version_change(self):
        source = release()
        cache = ReleaseValidationCache()
        result = cache.snapshot(source)
        result['payload']['version'] = 'private-edit'
        self.assertNotEqual(cache.snapshot(source)['payload']['version'], 'private-edit')
        source['payload']['version'] = 'valid-next-version'
        source['sha256'] = digest(source['payload'])
        self.assertEqual(cache.snapshot(source)['payload']['version'], 'valid-next-version')

    def test_alignment_skip_is_strict_and_only_pretask(self):
        step = dict(kind='join', label='align to pinned task start', target={'joint': .2})
        self.assertTrue(alignment_already_satisfied(step, {'joint': .204}))
        for measured in ({}, {'joint': float('nan')}, {'joint': .206}):
            self.assertFalse(alignment_already_satisfied(step, measured))
        self.assertFalse(alignment_already_satisfied(dict(step, label='contact join'), {'joint': .2}))
        self.assertFalse(alignment_already_satisfied(dict(step, target={}), {}))


if __name__ == '__main__':
    unittest.main()
