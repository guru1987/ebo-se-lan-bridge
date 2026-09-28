import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))

from ebo_media import EboMediaClient


def test_media_entry_parses_robot_listing():
    assert EboMediaClient._entry("/DICM", "Fsnap_20260101.jpg\t12345\t0\t1700000000") == {
        "name": "snap_20260101.jpg", "path": "/DICM/snap_20260101.jpg",
        "type": "photo", "size": 12345, "modified": 1700000000,
    }


def test_media_entry_ignores_directories_and_thumbnails():
    assert EboMediaClient._entry("/RECORD", "DAVI_EDIT\t0") is None
    assert EboMediaClient._entry("/RECORD", "F.THUMBNAIL_A_AVI.jpg\t10") is None
