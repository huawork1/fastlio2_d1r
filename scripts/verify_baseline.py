#!/usr/bin/env python3
"""Verify uploaded sources against the recorded SHA256 manifest."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / "baseline_manifest.json").read_text(encoding="utf-8"))
failed = []
for relative, expected in manifest["files"].items():
    path = root / relative
    actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    if actual != expected["sha256"]:
        failed.append(relative)
if failed:
    raise SystemExit("Source mismatch:\n" + "\n".join(failed))
print(f"Verified {len(manifest['files'])} algorithm and integration files.")
