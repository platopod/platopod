#!/usr/bin/env python3
"""atak_clear — send tombstone CoT events to remove stuck markers from ATAK/iTAK.

Use this when iTAK/ATAK is showing markers that the platform is no longer
publishing — typically because a previous session emitted them with the
`<archive>` element (which makes them persist in ATAK's local database
across app restarts) and your new scenario doesn't include them.

Two modes:

  --uids platopod-civ-shopkeeper platopod-ied-device_alpha
      Tombstone specific UIDs.

  --all-platopod
      Tombstone every UID matching common platopod-* patterns:
        platopod-arena
        platopod-1 … platopod-200       (robots)
        platopod-1-cas … platopod-200-cas (casualties)
        platopod-civ-{shopkeeper,bystander_1,child,…}
        platopod-ied-{device_alpha,…}
        platopod-plume-gas-* (a range of likely thresholds)
        platopod-cover-*
      This is the "nuke from orbit" option for clearing iTAK between
      scenarios when you don't know exactly what's stuck.

Usage:
  python3 tools/atak_clear.py --host 192.168.1.233 --port 4242 --all-platopod
  python3 tools/atak_clear.py --host 192.168.1.233 --port 4242 \
      --uids platopod-civ-shopkeeper platopod-civ-bystander_1
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

# Allow running from anywhere in the repo
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "server" / "src"))

from plato_pod.cot_protocol import (   # noqa: E402
    make_stale_update_event,
    make_tombstone_event,
)


def common_platopod_uid_groups() -> list[tuple[str, list[str]]]:
    """Return (original_type, [uids]) pairs for known platopod CoT shapes.

    The original type is needed for the same-UID-stale-update path,
    which iTAK honours when it ignores plain `t-x-d-d` deletes.
    """
    groups: list[tuple[str, list[str]]] = []

    # Arena boundary — drawing rectangle
    groups.append(("u-d-r", ["platopod-arena"]))

    # Robots — friendly ground (a-f-G family) and hostile (a-h-G family).
    # Send both type guesses since spawn-time team determines which.
    robot_uids = [f"platopod-{rid}" for rid in range(0, 200)]
    groups.append(("a-f-G", robot_uids))
    groups.append(("a-h-G", list(robot_uids)))
    # Vehicle-role-specific variants for a-f / a-h
    for variant in ("a-f-G-U-C-A", "a-f-G-U-C-I", "a-f-G-U-R", "a-f-G-E-S",
                    "a-h-G", "a-h-G-U-C-A", "a-h-G-X", "a-f-G-X"):
        groups.append((variant, list(robot_uids)))

    # Casualties
    cas_uids = [f"platopod-{rid}-cas" for rid in range(0, 200)]
    groups.append(("a-h-G-X", cas_uids))
    groups.append(("a-f-G-X", list(cas_uids)))

    # Civilians from the canonical scenarios — neutral ground
    civ_uids = [
        f"platopod-civ-{label}"
        for label in (
            "shopkeeper", "bystander_1", "bystander_2", "child", "elderly",
            "market", "civilian", "civilian_0", "civilian_1", "civilian_2",
            "civilian_3", "civilian_4", "civilian_5",
        )
    ]
    groups.append(("a-n-G", civ_uids))

    # IEDs — CBRN drawing
    ied_uids = [
        f"platopod-ied-{label}"
        for label in (
            "device_alpha", "device_bravo", "device_charlie",
            "ied", "ied_0", "ied_1", "ied_2", "ied_3", "ied_4",
        )
    ]
    groups.append(("u-d-c-c", ied_uids))

    # Plume contours — both polygon (u-d-f) and ellipse (u-d-c) renderers
    plume_uids: list[str] = []
    for field_name in ("gas", "radiation", "biological", "smoke"):
        for thr in (10, 50, 100, 200, 300, 500, 800, 1000, 1500, 2000):
            for comp_idx in range(0, 8):
                plume_uids.append(
                    f"platopod-plume-{field_name}-{thr}-{comp_idx}"
                )
    groups.append(("u-d-f", plume_uids))
    groups.append(("u-d-c", list(plume_uids)))
    groups.append(("u-d-r", list(plume_uids)))   # legacy emit type

    return groups


def common_platopod_uids() -> list[str]:
    """Flat list of all UIDs in common_platopod_uid_groups."""
    seen: set[str] = set()
    out: list[str] = []
    for _, uids in common_platopod_uid_groups():
        for u in uids:
            if u not in seen:
                seen.add(u)
                out.append(u)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True,
                        help="Target host (iTAK / ATAK device IP, or FTS)")
    parser.add_argument("--port", type=int, default=4242,
                        help="Target UDP port (default: 4242, ATAK default)")
    parser.add_argument("--uids", nargs="*", default=[],
                        help="Specific UIDs to tombstone")
    parser.add_argument("--type-for-uids", default="a-f-G",
                        help="Original type to assume for --uids (only "
                             "used for stale-update path)")
    parser.add_argument("--all-platopod", action="store_true",
                        help="Tombstone all common platopod-* UIDs across "
                             "every known type guess")
    parser.add_argument("--delay-ms", type=int, default=5,
                        help="Pause between sends (default: 5ms)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    target_count = 0

    def _send(xml: str, label: str) -> None:
        nonlocal sent
        try:
            sock.sendto(xml.encode("utf-8"), (args.host, args.port))
            sent += 1
        except Exception as e:
            print(f"  failed for {label}: {e}", file=sys.stderr)
        if args.delay_ms > 0:
            time.sleep(args.delay_ms / 1000.0)

    if args.all_platopod:
        groups = common_platopod_uid_groups()
        target_count = sum(len(uids) * 2 for _, uids in groups)
        print(f"Sending {target_count} delete events "
              f"(tombstone + stale-update per UID/type)")
        seen: set[str] = set()
        for original_type, uids in groups:
            for uid in uids:
                if uid not in seen:
                    _send(make_tombstone_event(uid), uid)
                    seen.add(uid)
                _send(make_stale_update_event(uid, original_type),
                      f"{uid}/{original_type}")

    if args.uids:
        target_count += len(args.uids) * 2
        print(f"Sending {len(args.uids) * 2} delete events for explicit UIDs")
        for uid in args.uids:
            _send(make_tombstone_event(uid), uid)
            _send(make_stale_update_event(uid, args.type_for_uids), uid)

    sock.close()
    if not target_count:
        print("No UIDs to tombstone — pass --uids or --all-platopod",
              file=sys.stderr)
        return 1
    print(f"Sent {sent}/{target_count} delete events")
    return 0


if __name__ == "__main__":
    sys.exit(main())
