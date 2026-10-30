import csv
import json
from pathlib import Path

from manager import write_csv, write_jsonl


def test_jsonl_writes_one_parseable_record_per_line(tmp_path: Path):
    path = tmp_path / "events.jsonl"
    write_jsonl(path, {"event_id": "a", "missing": None})
    write_jsonl(path, {"event_id": "b", "tags": ["ramp", "wet_surface"]})
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["event_id"] for row in rows] == ["a", "b"]
    assert rows[0]["missing"] is None


def test_csv_header_written_once_and_empty_values_preserved(tmp_path: Path):
    path = tmp_path / "sensor.csv"
    write_csv(path, ["seq", "valid", "distance_cm"], [1, True, 123])
    write_csv(path, ["seq", "valid", "distance_cm"], [2, False, None])
    with path.open(newline="") as fp:
        rows = list(csv.reader(fp))
    assert rows == [["seq", "valid", "distance_cm"], ["1", "True", "123"], ["2", "False", ""]]
