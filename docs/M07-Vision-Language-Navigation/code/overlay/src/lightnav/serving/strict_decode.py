"""Optional fixed-checkpoint acceptance gate; RVQ mathematics remains unchanged."""
import hashlib
import re

import numpy as np


class InvalidModelOutput(ValueError):
    def __init__(self, message: str, raw_text: str):
        super().__init__(message)
        self.raw_text = raw_text


def validate_tokens(text: str) -> dict:
    def reject(message):
        raise InvalidModelOutput(message, text)

    tokens = re.findall(r"<act_l[^>]*>", text)
    matches = [re.fullmatch(r"<act_l([0-2])_(\d+)>", token) for token in tokens]
    if len(tokens) != 3 or any(match is None for match in matches):
        reject("Expected exactly three well-formed RVQ action tokens")
    codes = {}
    for match in matches:
        level, code = map(int, match.groups())
        if level in codes or not 0 <= code < 256:
            reject("Duplicate RVQ level or out-of-range code")
        codes[level] = code
    grounding = {}
    for channel, maximum in [("apos", 1299), ("opos", 1296)]:
        tokens = re.findall(r"<" + channel + r"[^>]*>", text)
        match = re.fullmatch(r"<" + channel + r"_(\d+)>", tokens[0]) if len(tokens) == 1 else None
        if match is None or not 0 <= int(match.group(1)) <= maximum:
            reject(f"Missing, duplicate, malformed or out-of-range {channel}")
        grounding[channel + "_id"] = int(match.group(1))
    return {"codes": [codes[level] for level in range(3)], **grounding,
            "raw_text_chars": len(text), "raw_text_sha256": hashlib.sha256(text.encode()).hexdigest()}


def validate_decoded(text: str, waypoints: np.ndarray, info: dict, *, explicit_stop: bool) -> dict:
    if waypoints.shape != (10, 3) or waypoints.dtype != np.float32 or not np.isfinite(waypoints).all():
        raise InvalidModelOutput("RVQ trajectory must be finite float32 (10,3)", text)
    zero = bool(np.all(waypoints == 0))
    if info["apos_id"] == 1299 and not zero:
        raise InvalidModelOutput("APOS stop conflicts with decoded motion", text)
    return {**info, "model_output_valid": True,
            "decode_reason": "explicit_l0_stop" if explicit_stop else "near_zero_snap" if zero else "decoded_motion"}
