import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from gravity_console.shooting_groups import ShootingGroupStore, atomic_write_json


class ShootingGroupStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "groups.json"

    def store(self, legacy=None):
        return ShootingGroupStore(self.path, legacy or [])

    def test_defaults_and_snapshot_are_independent(self):
        store = self.store()
        self.assertEqual(len(store.groups), 3)
        self.assertTrue(all(len(g["blocks"]) == 4 and g["interval_s"] == 5 for g in store.groups))
        snapshot = store.snapshot([])
        snapshot[0]["name"] = "changed"
        self.assertNotEqual(snapshot[0]["name"], store.groups[0]["name"])
        self.assertEqual(store.groups, self.store().groups)

    def test_legacy_migration_preserves_twenty_slots_and_source(self):
        legacy = [{"slot": i, "samples": [{"q_rad": [0] * 6}]} for i in range(20, 0, -1)]
        before = copy.deepcopy(legacy)
        store = self.store(legacy)
        self.assertEqual([b["slot"] for b in store.groups[0]["blocks"]], list(range(1, 21)))
        self.assertEqual(legacy, before)
        group_id = store.groups[0]["id"]
        store.delete_block(group_id, store.groups[0]["blocks"][0]["id"])
        self.assertNotIn(1, [b["slot"] for b in self.store(legacy).groups[0]["blocks"]])

    def test_mutations_persist_and_summary_is_derived(self):
        store = self.store()
        gid = store.add_group()
        bid = store.add_block(gid)
        store.update_group(gid, "产品 A", 2.5)
        store.update_block(gid, bid, "镜头 A")
        store.assign_slot(gid, bid, 400)
        self.assertEqual(self.store().groups, store.groups)
        summary = {"slot": 400, "duration_s": 7}
        block = next(b for b in store.snapshot([summary])[-1]["blocks"] if b["id"] == bid)
        self.assertEqual(block["trajectory"], summary)
        self.assertNotIn("trajectory", store.resolve_block(gid, bid))
        store.delete_block(gid, bid)
        store.delete_group(gid)
        self.assertEqual(self.store().groups, store.groups)

    def test_at_least_one_group_and_block(self):
        store = self.store()
        while len(store.groups) > 1:
            store.delete_group(store.groups[-1]["id"])
        group = store.groups[0]
        while len(group["blocks"]) > 1:
            store.delete_block(group["id"], group["blocks"][-1]["id"])
            group = store.groups[0]
        with self.assertRaises(ValueError):
            store.delete_group(group["id"])
        with self.assertRaises(ValueError):
            store.delete_block(group["id"], group["blocks"][0]["id"])

    def test_limits_and_invalid_updates_leave_state_unchanged(self):
        store = self.store()
        gid = store.groups[0]["id"]
        old = copy.deepcopy(store.groups)
        for interval in (float("nan"), float("inf"), -1, 61, True, "5"):
            with self.assertRaises(ValueError):
                store.update_group(gid, interval_s=interval)
        for name in ("", " ", "x" * 81, 9):
            with self.assertRaises(ValueError):
                store.update_group(gid, name=name)
        for slot in (0, 401, True, 1.2):
            with self.assertRaises(ValueError):
                store.assign_slot(gid, old[0]["blocks"][0]["id"], slot)
        self.assertEqual(store.groups, old)
        while len(store.resolve_group(gid)["blocks"]) < 20:
            store.add_block(gid)
        with self.assertRaises(ValueError):
            store.add_block(gid)
        while len(store.groups) < 20:
            store.add_group()
        with self.assertRaises(ValueError):
            store.add_group()

    def test_duplicate_slots_and_ids_rejected(self):
        store = self.store()
        gid = store.groups[0]["id"]
        first, second = store.groups[0]["blocks"][:2]
        store.assign_slot(gid, first["id"], 1)
        with self.assertRaises(ValueError):
            store.assign_slot(gid, second["id"], 1)
        raw = json.loads(self.path.read_text())
        raw["groups"][0]["blocks"][1]["id"] = first["id"]
        self.path.write_text(json.dumps(raw))
        before = self.path.read_bytes()
        with self.assertRaises(ValueError):
            self.store()
        self.assertEqual(self.path.read_bytes(), before)

    def test_corrupt_file_never_overwritten(self):
        self.path.write_text("{broken")
        with self.assertRaises(ValueError):
            self.store()
        self.assertEqual(self.path.read_text(), "{broken")

    def test_schema_rejects_invalid_version_and_interval(self):
        self.store()
        original = json.loads(self.path.read_text())
        for version in (2, True, "1"):
            raw = copy.deepcopy(original)
            raw["version"] = version
            self.path.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):
                self.store()
        raw = copy.deepcopy(original)
        raw["groups"][0]["interval_s"] = float("nan")
        self.path.write_text(json.dumps(raw))
        with self.assertRaises(ValueError):
            self.store()

    def test_atomic_failure_keeps_disk_and_memory(self):
        store = self.store()
        before = self.path.read_bytes()
        groups = copy.deepcopy(store.groups)
        with patch("gravity_console.shooting_groups.os.replace", side_effect=OSError("disk failure")):
            with self.assertRaises(OSError):
                store.update_group(groups[0]["id"], name="new name")
        self.assertEqual(store.groups, groups)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])
        with self.assertRaises(ValueError):
            atomic_write_json(self.path, {"invalid": float("nan")})
        self.assertEqual(self.path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
