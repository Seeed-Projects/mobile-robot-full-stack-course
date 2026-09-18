"""BoundedStampCache for timestamp-aligned visualisation.

Default policy is exact-stamp matching. `tolerance_ns > 0` is permitted
ONLY when an audit documents an upstream stamp bug; the wrapper that
uses this cache is expected to log a one-time WARNING at startup
naming the upstream topic.

Public class:
    BoundedStampCache(maxlen=10, tolerance_ns=0)
        push(stamp_ns, msg)
        lookup(image_stamp_ns) -> ('exact', msg) | ('nearest', msg, age_ns) | (None,)
        __len__() / clear()
"""
from __future__ import annotations

from collections import OrderedDict
from typing import Any, NamedTuple, Optional, Tuple


class _Exact(NamedTuple):
    kind: str           # 'exact' or 'nearest'
    msg: Any
    age_ns: int = 0     # only meaningful for 'nearest'


_EXACT = _Exact(kind='exact', msg=None, age_ns=0)


class BoundedStampCache:
    """LRU cache keyed by header.stamp (ns); bounded size, no unbounded retention.

    lookup(image_stamp_ns):
        ('exact', msg)            — exact match
        ('nearest', msg, age_ns)  — only if tolerance_ns > 0 AND audit-blessed
        (None,)                   — no match in cache
    """

    def __init__(self, maxlen: int = 10, tolerance_ns: int = 0) -> None:
        if maxlen < 1:
            raise ValueError('maxlen must be >=1')
        if tolerance_ns < 0:
            raise ValueError('tolerance_ns must be >=0')
        self._maxlen = int(maxlen)
        self._tolerance_ns = int(tolerance_ns)
        self._cache: "OrderedDict[int, Any]" = OrderedDict()

    @property
    def maxlen(self) -> int:
        return self._maxlen

    @property
    def tolerance_ns(self) -> int:
        return self._tolerance_ns

    def __len__(self) -> int:
        return len(self._cache)

    def clear(self) -> None:
        self._cache.clear()

    def push(self, stamp_ns: int, msg: Any) -> None:
        if stamp_ns in self._cache:
            # Refresh position; new value wins (overwrite).
            self._cache.pop(stamp_ns)
        self._cache[stamp_ns] = msg
        while len(self._cache) > self._maxlen:
            self._cache.popitem(last=False)

    def lookup(self, image_stamp_ns: int) -> Tuple:
        """Return ('exact', msg) | ('nearest', msg, age_ns) | (None,)."""
        if image_stamp_ns in self._cache:
            # LRU bump
            v = self._cache.pop(image_stamp_ns)
            self._cache[image_stamp_ns] = v
            return ('exact', v)
        if self._tolerance_ns <= 0 or not self._cache:
            return (None,)
        # Nearest within tolerance. We do NOT pick a future stamp.
        best_ns: Optional[int] = None
        best_age = None
        for stamp in self._cache.keys():
            if stamp > image_stamp_ns:
                continue   # future — never used
            age = image_stamp_ns - stamp
            if age > self._tolerance_ns:
                continue
            if best_age is None or age < best_age:
                best_age = age
                best_ns = stamp
        if best_ns is None:
            return (None,)
        return ('nearest', self._cache[best_ns], int(best_age))
