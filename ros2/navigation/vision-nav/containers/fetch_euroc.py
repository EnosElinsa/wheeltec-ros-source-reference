#!/usr/bin/env python3
"""Fetch selected original sequence ZIPs from a pinned EuRoC mirror archive.

ETH's collection may rate limit downloads. This records the independent mirror
and validates ZIP CRC plus local SHA256; it does not claim ETH signature trust.
Only the selected nested ZIP is downloaded, using bounded HTTP byte ranges.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import struct
import subprocess
import zipfile
import zlib

REVISION = "29d08aeb1c8d9dc5c30497cc538d9cc61f872bb7"
URL = f"https://huggingface.co/datasets/GlowBond/EuRoC_MAV_Dataset/resolve/{REVISION}/machine_hall.zip"
SIZE = 12683729426


def byte_range(start, end, output):
    if output.exists() and output.stat().st_size == end - start + 1:
        return output
    temporary = output.with_suffix(output.suffix + ".partial")
    subprocess.run(["curl", "-fLsS", "--retry", "4", "--retry-all-errors", "--connect-timeout", "20",
                    "--max-time", "600", "--range", f"{start}-{end}",
                    URL, "-o", str(temporary)], check=True)
    if temporary.stat().st_size != end - start + 1:
        raise ValueError("Server did not return the requested byte range")
    temporary.replace(output)
    return output


class RemoteArchive(io.RawIOBase):
    def __init__(self, cache):
        self.position = 0
        self.cache = cache

    def seekable(self):
        return True

    def seek(self, offset, whence=0):
        self.position = offset + (SIZE if whence == 2 else self.position if whence == 1 else 0)
        return self.position

    def tell(self):
        return self.position

    def read(self, count=-1):
        end = SIZE if count < 0 else min(SIZE, self.position + count)
        if end <= self.position:
            return b""
        path = byte_range(self.position, end - 1, self.cache / f"range-{self.position}-{end}")
        self.position = end
        return path.read_bytes()


def fetch(sequence, destination, workers):
    destination.mkdir(parents=True, exist_ok=True)
    cache = destination / (".download-" + sequence)
    cache.mkdir(exist_ok=True)
    output = destination / (sequence + ".zip")
    if output.exists():
        raise FileExistsError(output)
    if (destination / sequence).exists():
        raise FileExistsError(destination / sequence)
    archive = RemoteArchive(cache)
    with zipfile.ZipFile(archive) as outer:
        info = outer.getinfo(f"machine_hall/{sequence}/{sequence}.zip")
    archive.seek(info.header_offset)
    header = archive.read(30)
    if header[:4] != b"PK\x03\x04" or info.compress_type not in (0, 8):
        raise ValueError("Unsupported mirror ZIP member")
    names, extra = struct.unpack_from("<HH", header, 26)
    start = info.header_offset + 30 + names + extra
    chunk_size = 8 * 1024 * 1024
    chunks = [(n, min(n + chunk_size, start + info.compress_size) - 1)
              for n in range(start, start + info.compress_size, chunk_size)]

    def download(bounds):
        lo, hi = bounds
        return byte_range(lo, hi, cache / f"chunk-{lo}-{hi}")

    with ThreadPoolExecutor(max_workers=workers) as pool:
        paths = list(pool.map(download, chunks))
    decompressor = zlib.decompressobj(-15) if info.compress_type == 8 else None
    crc, size = 0, 0
    sha = hashlib.sha256()
    partial = output.with_suffix(".zip.partial")
    with partial.open("wb") as stream:
        for path in paths:
            raw = path.read_bytes()
            data = decompressor.decompress(raw) if decompressor else raw
            stream.write(data)
            crc = zlib.crc32(data, crc)
            size += len(data)
            sha.update(data)
        if decompressor:
            data = decompressor.flush()
            stream.write(data)
            crc = zlib.crc32(data, crc)
            size += len(data)
            sha.update(data)
    if size != info.file_size or crc != info.CRC or (decompressor and not decompressor.eof):
        raise ValueError("Nested ZIP CRC/size validation failed")
    with zipfile.ZipFile(partial) as dataset:
        for member in dataset.infolist():
            target = (destination / sequence / member.filename).resolve()
            if not target.is_relative_to((destination / sequence).resolve()):
                raise ValueError("Unsafe dataset path")
        failed = dataset.testzip()
        if failed:
            raise ValueError("Dataset ZIP integrity failure: " + failed)
        dataset.extractall(destination / sequence)
    partial.replace(output)
    evidence = {"sequence": sequence, "source": URL, "mirror_revision": REVISION,
                "outer_member": info.filename, "outer_member_crc32": f"{info.CRC:08x}",
                "zip_bytes": size, "zip_sha256": sha.hexdigest(),
                "validation": "outer member CRC/size and all inner ZIP member CRCs",
                "official_source_equivalence": "not independently authenticated"}
    (destination / (sequence + "-download.json")).write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--sequence", choices=["MH_01_easy", "MH_03_medium", "MH_05_difficult"], required=True)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    if not 1 <= args.workers <= 16:
        parser.error("workers must be in [1,16]")
    fetch(args.sequence, args.destination.resolve(), args.workers)
