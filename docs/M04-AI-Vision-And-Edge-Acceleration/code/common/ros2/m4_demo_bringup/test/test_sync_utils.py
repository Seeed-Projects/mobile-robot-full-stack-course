"""Unit tests for m4_demo_bringup.sync_utils.BoundedStampCache.

Run with: pytest-3 modules/m04-ai-vision-and-edge-acceleration/common/ros2/m4_demo_bringup/test/test_sync_utils.py
"""
from m4_demo_bringup.sync_utils import BoundedStampCache


def test_exact_match_default():
    c = BoundedStampCache(maxlen=4, tolerance_ns=0)
    c.push(100, 'mA')
    c.push(200, 'mB')
    assert c.lookup(200) == ('exact', 'mB')
    assert c.lookup(100) == ('exact', 'mA')


def test_no_match_default_tolerance_zero():
    c = BoundedStampCache(maxlen=4, tolerance_ns=0)
    c.push(100, 'mA')
    # stamp 150 doesn't exist exactly -> no match
    assert c.lookup(150) == (None,)


def test_no_match_future_stamp():
    """Cache must never pick a future stamp on the image."""
    c = BoundedStampCache(maxlen=4, tolerance_ns=200)
    c.push(200, 'mB')
    # image is older than cached stamp; should NOT return mB
    assert c.lookup(100) == (None,)


def test_nearest_within_tolerance():
    c = BoundedStampCache(maxlen=4, tolerance_ns=300)
    c.push(100, 'mA')
    c.push(300, 'mC')
    # image=200 -> nearest cache entry within 300 ns tolerance = mA (age=100)
    res = c.lookup(200)
    assert res[0] == 'nearest'
    assert res[1] == 'mA'
    assert res[2] == 100


def test_nearest_outside_tolerance_returns_none():
    c = BoundedStampCache(maxlen=4, tolerance_ns=50)
    c.push(100, 'mA')
    # image=200 -> nearest cache entry would be mA but age=100 > 50
    assert c.lookup(200) == (None,)


def test_bounded_eviction():
    c = BoundedStampCache(maxlen=2)
    c.push(1, 'a')
    c.push(2, 'b')
    c.push(3, 'c')
    assert len(c) == 2
    assert c.lookup(1) == (None,)
    assert c.lookup(2) == ('exact', 'b')
    assert c.lookup(3) == ('exact', 'c')


def test_overwrite_same_stamp():
    c = BoundedStampCache(maxlen=4)
    c.push(100, 'old')
    c.push(100, 'new')
    assert len(c) == 1
    assert c.lookup(100) == ('exact', 'new')


def test_lru_bump():
    c = BoundedStampCache(maxlen=2)
    c.push(1, 'a')
    c.push(2, 'b')
    c.lookup(1)              # bumps a
    c.push(3, 'c')           # evicts oldest = b (a was bumped)
    assert c.lookup(2) == (None,)
    assert c.lookup(1) == ('exact', 'a')
    assert c.lookup(3) == ('exact', 'c')
