from threading import Event

from wheeltec_vision_nav.capture_runtime_logic import BoundedWriter, write_image_preview


def test_preview_queue_is_bounded_and_drains_on_close(tmp_path):
    release = Event()
    writer = BoundedWriter(capacity=1)
    try:
        assert writer.submit(release.wait, 2)
        assert not writer.submit(lambda: None)
    finally:
        release.set()
        writer.close()
    assert not writer.errors


def test_preview_preserves_rows_with_padding_and_collects_write_failure(tmp_path):
    path = tmp_path / "preview.png"
    write_image_preview(path, 2, 2, "mono8", 3, False, bytes([1, 2, 99, 3, 4, 99]))
    assert path.read_bytes().startswith(b"\x89PNG")
    writer = BoundedWriter()
    assert writer.submit(write_image_preview, path, 4, 4, "mono8", 4, False, b"short")
    writer.close()
    assert len(writer.errors) == 1 and "malformed" in writer.errors[0]
