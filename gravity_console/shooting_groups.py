"""Persistent shooting-group metadata, independent of trajectory sample storage."""
from __future__ import annotations

import copy
import json
import math
import os
import tempfile
import uuid
from pathlib import Path
from typing import Any

MAX_GROUPS = 20
MAX_BLOCKS = 20
MAX_SLOTS = 400


def atomic_write_json(path: Path, data: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = stream.name
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


class ShootingGroupStore:
    def __init__(self, path: Path, legacy_trajectories: list[dict[str, Any]]) -> None:
        self.path = Path(path)
        self.groups: list[dict[str, Any]] = []
        if self.path.exists():
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(raw, dict) or type(raw.get("version")) is not int or raw["version"] != 1:
                raise ValueError("unsupported shooting group file version")
            if raw.get("legacy_imported") is not True:
                raise ValueError("shooting group migration marker missing")
            groups = raw.get("groups")
            self._validate(groups)
            self.groups = copy.deepcopy(groups)
        else:
            groups = [self._new_group(i + 1) for i in range(3)]
            slots: list[int] = []
            for trajectory in legacy_trajectories:
                slot = trajectory.get("slot")
                self._validate_slot(slot)
                if slot in slots:
                    raise ValueError("duplicate legacy trajectory slot")
                slots.append(slot)
            slots.sort()
            if len(slots) > MAX_BLOCKS:
                raise ValueError("too many legacy trajectories for first shooting group")
            while len(groups[0]["blocks"]) < len(slots):
                groups[0]["blocks"].append(self._new_block(len(groups[0]["blocks"]) + 1))
            for block, slot in zip(groups[0]["blocks"], slots):
                block["slot"] = slot
            self._commit(groups)

    @staticmethod
    def _new_block(number: int) -> dict[str, Any]:
        return {"id": str(uuid.uuid4()), "name": f"动作 {number}", "slot": None}

    @classmethod
    def _new_group(cls, number: int) -> dict[str, Any]:
        return {"id": str(uuid.uuid4()), "name": f"拍摄组 {number}", "interval_s": 5.0,
                "blocks": [cls._new_block(i + 1) for i in range(4)]}

    @staticmethod
    def _validate_slot(slot: Any) -> None:
        if type(slot) is not int or not 1 <= slot <= MAX_SLOTS:
            raise ValueError(f"trajectory slot must be 1-{MAX_SLOTS}")

    @staticmethod
    def _validate_name(name: Any) -> None:
        if not isinstance(name, str) or not name.strip() or len(name) > 80:
            raise ValueError("name must contain 1-80 characters")

    @classmethod
    def _validate(cls, groups: Any) -> None:
        if not isinstance(groups, list) or not 1 <= len(groups) <= MAX_GROUPS:
            raise ValueError(f"shooting groups must contain 1-{MAX_GROUPS} groups")
        ids: set[str] = set()
        slots: set[int] = set()
        for group in groups:
            if not isinstance(group, dict):
                raise ValueError("invalid shooting group")
            cls._validate_id(group.get("id"), ids)
            cls._validate_name(group.get("name"))
            interval = group.get("interval_s")
            if type(interval) not in (int, float) or not math.isfinite(interval) or not 0 <= interval <= 60:
                raise ValueError("group interval must be between 0 and 60 seconds")
            blocks = group.get("blocks")
            if not isinstance(blocks, list) or not 1 <= len(blocks) <= MAX_BLOCKS:
                raise ValueError(f"each shooting group must contain 1-{MAX_BLOCKS} blocks")
            for block in blocks:
                if not isinstance(block, dict):
                    raise ValueError("invalid action block")
                cls._validate_id(block.get("id"), ids)
                cls._validate_name(block.get("name"))
                if "slot" not in block:
                    raise ValueError("action block slot missing")
                slot = block["slot"]
                if slot is not None:
                    cls._validate_slot(slot)
                    if slot in slots:
                        raise ValueError("duplicate trajectory slot reference")
                    slots.add(slot)

    @staticmethod
    def _validate_id(identifier: Any, ids: set[str]) -> None:
        if not isinstance(identifier, str):
            raise ValueError("invalid group or block id")
        try:
            if str(uuid.UUID(identifier)) != identifier:
                raise ValueError("noncanonical id")
        except (ValueError, AttributeError) as exc:
            raise ValueError("invalid group or block id") from exc
        if identifier in ids:
            raise ValueError("duplicate group or block id")
        ids.add(identifier)

    def _commit(self, groups: list[dict[str, Any]]) -> None:
        self._validate(groups)
        atomic_write_json(self.path, {"version": 1, "legacy_imported": True, "groups": groups})
        self.groups = groups

    @staticmethod
    def _group(groups: list[dict[str, Any]], group_id: str) -> dict[str, Any]:
        for group in groups:
            if group["id"] == group_id:
                return group
        raise ValueError("shooting group not found")

    @classmethod
    def _block(cls, groups: list[dict[str, Any]], group_id: str, block_id: str) -> dict[str, Any]:
        for block in cls._group(groups, group_id)["blocks"]:
            if block["id"] == block_id:
                return block
        raise ValueError("action block not found")

    def resolve_group(self, group_id: str) -> dict[str, Any]:
        return self._group(self.groups, group_id)

    def resolve_block(self, group_id: str, block_id: str) -> dict[str, Any]:
        return self._block(self.groups, group_id, block_id)

    def add_group(self) -> str:
        groups = copy.deepcopy(self.groups)
        group = self._new_group(len(groups) + 1)
        groups.append(group)
        self._commit(groups)
        return group["id"]

    def update_group(self, group_id: str, name: str | None = None, interval_s: float | None = None) -> None:
        groups = copy.deepcopy(self.groups)
        group = self._group(groups, group_id)
        if name is not None:
            group["name"] = name
        if interval_s is not None:
            group["interval_s"] = interval_s
        self._commit(groups)

    def delete_group(self, group_id: str) -> None:
        self.resolve_group(group_id)
        self._commit(copy.deepcopy([g for g in self.groups if g["id"] != group_id]))

    def add_block(self, group_id: str) -> str:
        groups = copy.deepcopy(self.groups)
        blocks = self._group(groups, group_id)["blocks"]
        block = self._new_block(len(blocks) + 1)
        blocks.append(block)
        self._commit(groups)
        return block["id"]

    def update_block(self, group_id: str, block_id: str, name: str) -> None:
        groups = copy.deepcopy(self.groups)
        self._block(groups, group_id, block_id)["name"] = name
        self._commit(groups)

    def delete_block(self, group_id: str, block_id: str) -> None:
        groups = copy.deepcopy(self.groups)
        self._block(groups, group_id, block_id)
        group = self._group(groups, group_id)
        group["blocks"] = [b for b in group["blocks"] if b["id"] != block_id]
        self._commit(groups)

    def assign_slot(self, group_id: str, block_id: str, slot: int) -> None:
        self._validate_slot(slot)
        groups = copy.deepcopy(self.groups)
        self._block(groups, group_id, block_id)["slot"] = slot
        self._commit(groups)

    def snapshot(self, trajectory_summaries: list[dict[str, Any]]) -> list[dict[str, Any]]:
        groups = copy.deepcopy(self.groups)
        by_slot = {t["slot"]: t for t in trajectory_summaries}
        for group in groups:
            for block in group["blocks"]:
                block["trajectory"] = copy.deepcopy(by_slot.get(block["slot"]))
        return groups
