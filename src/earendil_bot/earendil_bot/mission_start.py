#!/usr/bin/env python3
"""Mission Start — Sequential Autonomous Mission Runner

Runs individual ROS 2 mission nodes one after another.
Each node must terminate itself with sys.exit(0) when its task is done.
The coordinator waits for each to finish before launching the next.

Mission Sequence:
    1. base_exit          — Exit the starting enclosure, save home position
    2. mission_nav        — Interactive menu to select and navigate to a waypoint
    3. tunnel_aruco_nav   — Detect ArUco markers, drive through tunnel
    4. base_return        — Return to the saved starting position

Usage:
    ros2 run earendil_bot mission_start
"""

import subprocess
import sys
import time


# ─────────────────────────────────────────────
#  Mission Sequence — edit this list to change order
# ─────────────────────────────────────────────
MISSION_SEQUENCE = [
    ("base_exit",        "Exit enclosure & save home position"),
    ("mission_nav",      "Navigate to mission waypoint (interactive)"),
    ("tunnel_aruco_nav", "ArUco tunnel traversal"),
    ("base_return",      "Return to home base"),
]

# If True, continue to the next mission even if one fails.
# If False, abort everything on first failure.
CONTINUE_ON_FAILURE = False


def run_mission(executable: str, description: str, index: int, total: int) -> bool:
    """Run a single ROS 2 node as a subprocess and wait for it to finish.

    Returns True if the mission completed successfully.
    """
    print(f"\n{'='*60}")
    print(f"  [{index}/{total}] 🚀 {description}")
    print(f"  Executable: {executable}")
    print(f"{'='*60}\n")

    cmd = [
        "ros2", "run", "earendil_bot", executable,
        "--ros-args", "-p", "use_sim_time:=false"
    ]

    start_time = time.time()

    try:
        result = subprocess.run(cmd, check=False)
        elapsed = time.time() - start_time
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)

        if result.returncode == 0:
            print(f"\n  ✅ [{index}/{total}] {description} — SUCCESS ({minutes}m {seconds}s)")
            return True
        else:
            print(f"\n  ❌ [{index}/{total}] {description} — FAILED "
                  f"(exit code {result.returncode}, {minutes}m {seconds}s)")
            return False

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n  ⚠️  [{index}/{total}] {description} — INTERRUPTED by user "
              f"({int(elapsed)}s)")
        raise  # Re-raise to exit coordinator


def main():
    total = len(MISSION_SEQUENCE)
    completed = 0
    failed = 0

    print("\n" + "#" * 60)
    print("  🤖  AUTONOMOUS ROVER MISSION COORDINATOR")
    print("#" * 60)
    print(f"\n  Total missions: {total}")
    print(f"  Continue on failure: {CONTINUE_ON_FAILURE}\n")
    for i, (exe, desc) in enumerate(MISSION_SEQUENCE, 1):
        print(f"    {i}. {desc} ({exe})")
    print()

    overall_start = time.time()

    try:
        for i, (exe, desc) in enumerate(MISSION_SEQUENCE, 1):
            success = run_mission(exe, desc, i, total)

            if success:
                completed += 1
            else:
                failed += 1
                if not CONTINUE_ON_FAILURE:
                    print("\n  🛑 Aborting remaining missions due to failure.")
                    break

    except KeyboardInterrupt:
        print("\n\n  🛑 Mission coordinator interrupted by user.")

    # ── Summary ──
    overall_elapsed = time.time() - overall_start
    overall_min = int(overall_elapsed // 60)
    overall_sec = int(overall_elapsed % 60)
    skipped = total - completed - failed

    print(f"\n{'#'*60}")
    print(f"  📊 MISSION SUMMARY")
    print(f"{'#'*60}")
    print(f"  ✅ Completed : {completed}/{total}")
    print(f"  ❌ Failed    : {failed}/{total}")
    print(f"  ⏭️  Skipped   : {skipped}/{total}")
    print(f"  ⏱️  Total time: {overall_min}m {overall_sec}s")
    print(f"{'#'*60}\n")

    if failed > 0:
        sys.exit(1)
    else:
        print("  🎉 ALL MISSIONS COMPLETED SUCCESSFULLY!\n")
        sys.exit(0)


if __name__ == '__main__':
    main()
