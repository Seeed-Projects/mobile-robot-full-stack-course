#!/usr/bin/env python3
"""Download one pinned public checkpoint and verify every selected file."""
import argparse
import hashlib
import json
import os
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / ".jetson-lightnav"
MODEL = ROOT / "checkpoints/LightNav-0"
REPO_ID = "LightOriginsHQ/LightNav-0"
REVISION = "826dc5fbfa37afa8293d2e336d329b6ffc0bfb64"
WEIGHT_SHA = "ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true", help="Verify a transferred snapshot using saved official metadata")
    args = parser.parse_args()
    os.environ["HF_HUB_DISABLE_XET"] = "1"
    os.environ["HF_HOME"] = str(DATA / "hf-cache")
    from huggingface_hub import snapshot_download

    api = f"https://huggingface.co/api/models/{REPO_ID}/revision/{REVISION}?blobs=true"
    if args.verify_only:
        metadata = json.loads((DATA / "hf-source-metadata.json").read_text())
    else:
        with urllib.request.urlopen(api, timeout=60) as response:
            metadata = json.load(response)
    assert metadata["sha"] == REVISION
    selected = [entry for entry in metadata["siblings"]
                if entry["rfilename"] not in {".gitattributes", "wechat_group.png"}]
    assert len(selected) == 18, f"Unexpected pinned snapshot file count: {len(selected)}"
    (DATA / "hf-source-metadata.json").write_text(json.dumps(metadata, indent=2))
    if not args.verify_only:
        snapshot_download(REPO_ID, revision=REVISION, token=False, local_dir=MODEL,
                          allow_patterns=[entry["rfilename"] for entry in selected], max_workers=2)
    verified = []
    for entry in selected:
        path = MODEL / entry["rfilename"]
        size = path.stat().st_size
        assert size == entry["size"], f"Size mismatch: {path}"
        sha = hashlib.sha256()
        git_hash = hashlib.sha1(f"blob {size}\0".encode())
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(8 * 1024 * 1024), b""):
                sha.update(chunk)
                git_hash.update(chunk)
        actual = sha.hexdigest()
        if entry.get("lfs"):
            assert actual == entry["lfs"]["sha256"], f"LFS SHA mismatch: {path}"
        else:
            assert git_hash.hexdigest() == entry["blobId"], f"Git blob SHA mismatch: {path}"
        if path.suffix == ".safetensors":
            assert actual == WEIGHT_SHA, "Weight SHA differs from preflight"
        verified.append({"file": entry["rfilename"], "bytes": size, "sha256": actual,
                         "source_blob": entry["blobId"]})
    manifest = {"status": "PASS", "repo_id": REPO_ID, "revision": REVISION,
                "total_bytes": sum(item["bytes"] for item in verified), "files": verified}
    (DATA / "model-manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest), flush=True)


if __name__ == "__main__":
    main()
