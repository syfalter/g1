"""Validate fixed production releases once; keep mutable development loaders checked."""
import copy

from bundle import digest
from releases import validate


def _immutable(*args, **kwargs):
    raise TypeError('Pinned production release is immutable; activate another version to change it')


class FrozenDict(dict):
    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = __ior__ = _immutable

    def __deepcopy__(self, memo):
        result = {}
        memo[id(self)] = result
        result.update((k, copy.deepcopy(v, memo)) for k, v in self.items())
        return result


class FrozenList(list):
    __setitem__ = __delitem__ = append = clear = extend = insert = pop = remove = reverse = sort = __iadd__ = __imul__ = _immutable

    def __deepcopy__(self, memo):
        result = []
        memo[id(self)] = result
        result.extend(copy.deepcopy(v, memo) for v in self)
        return result


def _freeze(value):
    if isinstance(value, dict):
        return FrozenDict({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, list):
        return FrozenList(_freeze(v) for v in value)
    return value


class PinnedRelease:
    """Only constructed at service startup. Nested public containers reject mutation."""
    __slots__ = ('_snapshot',)

    def __init__(self, selected):
        object.__setattr__(self, '_snapshot', _freeze(validate(copy.deepcopy(selected))))

    __setattr__ = __delattr__ = _immutable

    @property
    def snapshot(self):
        return self._snapshot


class ReleaseValidationCache:
    def __init__(self):
        self.verified_sha = None

    def snapshot(self, selected):
        if isinstance(selected, PinnedRelease):
            return selected.snapshot
        snapshot = copy.deepcopy(selected)
        actual = digest(snapshot['payload'])
        if actual != snapshot['sha256']:
            raise ValueError('Selected release content changed without matching hash')
        if actual != self.verified_sha:
            validate(snapshot)
            self.verified_sha = actual
        # A fresh private copy prevents runtime or callers mutating the cached trust boundary.
        return snapshot
