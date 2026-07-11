from __future__ import annotations
import json
import stat

from calibration.store import load_store, write_store


def test_write_store_replaces_json_and_preserves_mode(tmp_path):
    path = tmp_path / "store.json"
    path.write_text('{"old": true}\n')
    path.chmod(0o640)
    before = path.stat()

    write_store(path, {"new": [1, 2, 3]})

    after = path.stat()
    assert load_store(path) == {"new": [1, 2, 3]}
    assert stat.S_IMODE(after.st_mode) == stat.S_IMODE(before.st_mode)
    assert (after.st_uid, after.st_gid) == (before.st_uid, before.st_gid)
    assert json.loads(path.read_text()) == {"new": [1, 2, 3]}


def test_load_store_returns_empty_when_absent(tmp_path):
    assert load_store(tmp_path / "missing.json") == {}
