"""Verify selected-member download integrity and extraction boundaries."""
import importlib.util
import io
import json
from pathlib import Path
import zipfile

import pytest

spec = importlib.util.spec_from_file_location("fetch_euroc", Path(__file__).parents[1] / "fetch_euroc.py")
fetcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fetcher)


def fake_mirror(monkeypatch, name="mav0/cam0/data.csv"):
    inner = io.BytesIO()
    with zipfile.ZipFile(inner, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(name, "#timestamp [ns],filename\n1,1.png\n")
    outer = io.BytesIO()
    with zipfile.ZipFile(outer, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("machine_hall/MH_01_easy/MH_01_easy.zip", inner.getvalue())
    payload = outer.getvalue()
    monkeypatch.setattr(fetcher, "SIZE", len(payload))

    def read_range(start, end, output):
        output.write_bytes(payload[start:end + 1])
        return output

    monkeypatch.setattr(fetcher, "byte_range", read_range)
    return inner.getvalue()


def test_nested_sequence_integrity_and_evidence(tmp_path, monkeypatch):
    original = fake_mirror(monkeypatch)
    fetcher.fetch("MH_01_easy", tmp_path, 2)
    assert (tmp_path / "MH_01_easy.zip").read_bytes() == original
    assert (tmp_path / "MH_01_easy/mav0/cam0/data.csv").is_file()
    evidence = json.loads((tmp_path / "MH_01_easy-download.json").read_text())
    assert evidence["zip_sha256"] == fetcher.hashlib.sha256(original).hexdigest()
    assert evidence["official_source_equivalence"] == "not independently authenticated"
    with pytest.raises(FileExistsError):
        fetcher.fetch("MH_01_easy", tmp_path, 2)


def test_unsafe_nested_path_is_never_extracted(tmp_path, monkeypatch):
    fake_mirror(monkeypatch, "../escaped.txt")
    with pytest.raises(ValueError, match="Unsafe dataset path"):
        fetcher.fetch("MH_01_easy", tmp_path, 2)
    assert not (tmp_path / "escaped.txt").exists()


def test_existing_unzipped_data_is_preserved(tmp_path, monkeypatch):
    fake_mirror(monkeypatch)
    sequence = tmp_path / "MH_01_easy"
    sequence.mkdir()
    sentinel = sequence / "original.txt"
    sentinel.write_text("keep")
    with pytest.raises(FileExistsError):
        fetcher.fetch("MH_01_easy", tmp_path, 2)
    assert sentinel.read_text() == "keep"
