# Extracted for offline teaching verification from fixed upstream processing.py.
# Original SHA256: 470c1bc84bce40648105492e5b1b657bfe5cc49fec1c192bed27649add18684c
import os

def _calculate_timestamps(self, indices, video_fps, merge_size: int=2):
    """Per-temporal-patch timestamps rendered as ``<X.X seconds>`` prefixes.

        Default (absolute) is byte-identical to the parent / every existing
        checkpoint: ``t = idx / fps`` (seconds since episode start).

        ``VLN_TIMESTAMP_RELATIVE=1`` switches to time-before-current
        ``t = (newest_idx - idx) / fps`` so the current frame anchors near 0 and
        the wording no longer drifts with episode length. This changes the learned
        input, so it MUST match between training and inference; the model loader
        bridges the checkpoint's recorded mode into this env var. The
        ``<X.X seconds>`` render format is unchanged either way.
        """
    if not isinstance(indices, list):
        indices = indices.tolist()
    if len(indices) % merge_size != 0:
        indices.extend((indices[-1] for _ in range(merge_size - len(indices) % merge_size)))
    if os.environ.get('VLN_TIMESTAMP_RELATIVE', '0').lower() in ('1', 'true', 'yes'):
        ref = getattr(self, '_ts_global_ref', None)
        if ref is None:
            ref = max(indices)
        secs = [(ref - idx) / video_fps for idx in indices]
    else:
        secs = [idx / video_fps for idx in indices]
    return [(secs[i] + secs[i + merge_size - 1]) / 2 for i in range(0, len(secs), merge_size)]
