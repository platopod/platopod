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

from plato_pod.cot_protocol import make_tombstone_event   # noqa: E402


def common_platopod_uids() -> list[str]:
    """Best-guess set of UIDs the platform might have published."""
    uids: list[str] = ["platopod-arena"]

    # Robots and casualties (cover a wide id range — auto-incrementing
    # spawns can quickly reach two-digit IDs over a development session)
    for rid in range(0, 200):
        uids.append(f"platopod-{rid}")
        uids.append(f"platopod-{rid}-cas")

    # Civilians from the canonical scenarios
    for label in (
        "shopkeeper", "bystander_1", "bystander_2", "child", "elderly",
        "market", "civilian", "civilian_0", "civilian_1", "civilian_2",
        "civilian_3", "civilian_4", "civilian_5",
    ):
        uids.append(f"platopod-civ-{label}")

    # IEDs from the canonical scenarios
    for label in (
        "device_alpha", "device_bravo", "device_charlie",
        "ied", "ied_0", "ied_1", "ied_2", "ied_3", "ied_4",
    ):
        uids.append(f"platopod-ied-{label}")

    # Plume contours — common field names × thresholds × component idx
    for field_name in ("gas", "radiation", "biological", "smoke"):
        for thr in (10, 50, 100, 200, 300, 500, 800, 1000, 1500, 2000):
            for comp_idx in range(0, 8):
                uids.append(f"platopod-plume-{field_name}-{thr}-{comp_idx}")

    # Engagement / fire markers (timestamp-based UIDs are random, can't
    # tombstone by name; rely on stale_seconds expiry instead).

    return uids


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True,
                        help="Target host (iTAK / ATAK device IP, or FTS)")
    parser.add_argument("--port", type=int, default=4242,
                        help="Target UDP port (default: 4242, ATAK default)")
    parser.add_argument("--uids", nargs="*", default=[],
                        help="Specific UIDs to tombstone")
    parser.add_argument("--all-platopod", action="store_true",
                        help="Tombstone all common platopod-* UIDs")
    parser.add_argument("--delay-ms", type=int, default=10,
                        help="Pause between sends (default: 10ms)")
    args = parser.parse_args()

    targets = list(args.uids)
    if args.all_platopod:
        targets.extend(common_platopod_uids())
    targets = sorted(set(targets))

    if not targets:
        print("No UIDs to tombstone — pass --uids or --all-platopod",
              file=sys.stderr)
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Tombstoning {len(targets)} UIDs to {args.host}:{args.port}")
    sent = 0
    for uid in targets:
        try:
            xml = make_tombstone_event(uid)
            sock.sendto(xml.encode("utf-8"), (args.host, args.port))
            sent += 1
        except Exception as e:
            print(f"  failed for {uid}: {e}", file=sys.stderr)
        if args.delay_ms > 0:
            time.sleep(args.delay_ms / 1000.0)
    sock.close()
    print(f"Sent {sent}/{len(targets)} tombstones")
    return 0


if __name__ == "__main__":
    sys.exit(main())
