#!/usr/bin/env python3
"""Download an official runtime archive in bounded, resumable HTTP ranges."""
import argparse
import concurrent.futures
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("url")
    parser.add_argument("output", type=Path)
    parser.add_argument("--connections", type=int, default=8)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(urllib.request.Request(args.url, method="HEAD"), timeout=30) as response:
        size = int(response.headers["Content-Length"])
        etag = response.headers.get("ETag")
    manifest = output.with_suffix(output.suffix + ".json")
    if output.exists():
        if output.stat().st_size == size and manifest.exists():
            print(f"Already downloaded: {output}", flush=True)
            return
        raise RuntimeError(f"Refusing to overwrite existing incomplete/unverified file: {output}")
    parts = output.with_suffix(output.suffix + ".parts")
    parts.mkdir(exist_ok=True)
    chunk = 64 * 1024 * 1024
    count = math.ceil(size / chunk)
    started = time.monotonic()

    def download(index):
        first = index * chunk
        last = min(size, first + chunk) - 1
        expected = last - first + 1
        part = parts / f"{index:04d}.part"
        if part.exists() and part.stat().st_size == expected:
            return expected
        for attempt in range(4):
            try:
                request = urllib.request.Request(args.url, headers={"Range": f"bytes={first}-{last}", "If-Range": etag or ""})
                with urllib.request.urlopen(request, timeout=60) as response, part.open("wb") as target:
                    if response.status != 206 or response.headers.get("Content-Range") != f"bytes {first}-{last}/{size}":
                        raise RuntimeError("Server did not honor the requested byte range")
                    shutil.copyfileobj(response, target, length=1024 * 1024)
                if part.stat().st_size != expected:
                    raise RuntimeError("Incomplete range download")
                return expected
            except Exception:
                if attempt == 3:
                    raise
                time.sleep(2 ** attempt)

    print(f"Downloading {size / 1024**3:.2f} GiB from {args.url}", flush=True)
    total = 0
    last_report = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.connections) as executor:
        for future in concurrent.futures.as_completed([executor.submit(download, i) for i in range(count)]):
            total += future.result()
            percent = int(100 * total / size)
            if percent >= last_report + 5 or total == size:
                print(f"{percent}% | {total / 1024**3:.2f} GiB | {time.monotonic()-started:.0f}s", flush=True)
                last_report = percent
    digest = hashlib.sha256()
    temporary = output.with_suffix(output.suffix + ".assembling")
    with temporary.open("wb") as target:
        for index in range(count):
            with (parts / f"{index:04d}.part").open("rb") as source:
                while block := source.read(8 * 1024 * 1024):
                    target.write(block)
                    digest.update(block)
    assert temporary.stat().st_size == size
    os.replace(temporary, output)
    manifest.write_text(json.dumps({"url": args.url, "size": size, "etag": etag, "sha256": digest.hexdigest()}, indent=2) + "\n")
    shutil.rmtree(parts)
    print(f"Complete: {output}\nSHA256: {digest.hexdigest()}", flush=True)


if __name__ == "__main__":
    main()
