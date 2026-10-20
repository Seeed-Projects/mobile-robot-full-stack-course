#!/usr/bin/env python3
"""Repair the upstream 0.7.1 ARM wheel's SBSA tag, without changing its payload."""
import base64
import csv
import hashlib
import io
import json
import os
import time
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / ".jetson-lightnav"
NAME = "nvidia_cusparselt_cu12-0.7.1-py3-none-manylinux2014_aarch64.whl"
URL = "https://pypi.nvidia.com/nvidia-cusparselt-cu12/" + NAME
ORIGINAL_SHA = "8878dce784d0fac90131b6817b607e803c36e629ba34dc5b433471382196b6a5"
DIST = "nvidia_cusparselt_cu12-0.7.1.dist-info/"


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            result.update(block)
    return result.hexdigest()


def prepare():
    original = DATA / "wheels/original" / NAME
    repaired = DATA / "wheels/repaired" / NAME
    manifest = DATA / "evidence/cusparselt-wheel-repair.json"
    for path in [original, repaired, manifest]:
        path.parent.mkdir(parents=True, exist_ok=True)
    if repaired.exists() and manifest.exists():
        recorded = json.loads(manifest.read_text())
        if digest(repaired) == recorded["repaired_sha256"]:
            return repaired, recorded["repaired_sha256"]
        raise RuntimeError("Previously repaired wheel hash mismatch")
    if not original.exists():
        temporary = original.with_suffix(".partial")
        last_check = 0
        with urllib.request.urlopen(URL, timeout=60) as response, temporary.open("wb") as output:
            while block := response.read(1024 * 1024):
                output.write(block)
                if time.monotonic() - last_check >= 15:
                    stat = os.statvfs(ROOT)
                    remaining = stat.f_bavail * stat.f_frsize
                    print(json.dumps({"downloaded": output.tell(), "free_bytes": remaining}), flush=True)
                    if remaining < 3 * 2**30:
                        raise RuntimeError("Disk reserve below 3 GiB")
                    last_check = time.monotonic()
        temporary.replace(original)
    if digest(original) != ORIGINAL_SHA:
        raise RuntimeError("Official NVIDIA ARM wheel SHA256 mismatch")
    payload_hashes = {}
    with zipfile.ZipFile(original) as source:
        before = source.read(DIST + "WHEEL")
        expected = b"Tag: py3-none-manylinux2014_sbsa"
        if before.count(expected) != 1:
            raise RuntimeError("Unexpected upstream WHEEL metadata; no repair applied")
        after = before.replace(expected, b"Tag: py3-none-manylinux2014_aarch64")
        rows = list(csv.reader(io.StringIO(source.read(DIST + "RECORD").decode())))
        indexed = {row[0]: row for row in rows}
        if len(indexed) != len(rows):
            raise RuntimeError("Duplicate upstream RECORD entry")
        repaired_record = []
        with zipfile.ZipFile(repaired.with_suffix(".partial"), "w") as output:
            for member in source.infolist():
                if member.filename.endswith("/"):
                    output.writestr(member, b"")
                    continue
                value = source.read(member)
                record = indexed[member.filename]
                if member.filename != DIST + "RECORD":
                    encoded = base64.urlsafe_b64encode(hashlib.sha256(value).digest()).rstrip(b"=").decode()
                    if record[1:] != ["sha256=" + encoded, str(len(value))]:
                        raise RuntimeError("Upstream RECORD mismatch: " + member.filename)
                if member.filename == DIST + "RECORD":
                    continue
                if member.filename == DIST + "WHEEL":
                    value = after
                else:
                    payload_hashes[member.filename] = hashlib.sha256(value).hexdigest()
                encoded = base64.urlsafe_b64encode(hashlib.sha256(value).digest()).rstrip(b"=").decode()
                repaired_record.append([member.filename, "sha256=" + encoded, str(len(value))])
                output.writestr(member, value)
            repaired_record.append([DIST + "RECORD", "", ""])
            stream = io.StringIO(newline="")
            csv.writer(stream, lineterminator="\n").writerows(repaired_record)
            output.writestr(DIST + "RECORD", stream.getvalue())
    repaired.with_suffix(".partial").replace(repaired)
    with zipfile.ZipFile(repaired) as output:
        for filename, expected_hash in payload_hashes.items():
            if hashlib.sha256(output.read(filename)).hexdigest() != expected_hash:
                raise RuntimeError("Payload changed during metadata repair: " + filename)
    repaired_sha = digest(repaired)
    manifest.write_text(json.dumps({"source_url": URL, "source_sha256": ORIGINAL_SHA,
                                   "repaired_sha256": repaired_sha,
                                   "wheel_before": before.decode(), "wheel_after": after.decode(),
                                   "changed_members": [DIST + "WHEEL", DIST + "RECORD"],
                                   "unchanged_payload_sha256": payload_hashes,
                                   "purpose": "Correct filename/internal platform tag mismatch; ABI remains subject to Gate 1",
                                   "time": time.time()}, indent=2))
    return repaired, repaired_sha


if __name__ == "__main__":
    print(json.dumps(dict(zip(["wheel", "sha256"], map(str, prepare())))))
