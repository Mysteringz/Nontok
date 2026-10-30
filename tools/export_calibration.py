"""Copy the console's pose captures into the repository so they travel with the code.

The console appends every capture to .local-data/calibration/pose-captures.jsonl, which git
ignores. This writes calibration/<name>.jsonl (every capture, raw samples included) and
calibration/<name>.json: the step-test targets the console would send as REF, worked out the
same way (the last capture per pose wins; a reading counts only if >= 90 % of samples had an
echo and its spread is under 25 mm).
Run: python3 tools/export_calibration.py field-2026-10-30
"""
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / ".local-data" / "calibration" / "pose-captures.jsonl"
POSES = ("p_point", "centre", "turned_left", "left_box", "turned_right", "right_box")
REF_KEYS = ("front", "side_left", "side_right", "left", "right")  # REF order: front ls rs l45 r45
NO_REF = 9999


def steady(capture, key):
    st = capture["stats"].get("us_%s_mm" % key, {})
    if st.get("n", 0) >= 0.9 * capture["samples_n"] and st.get("sd", 99) < 25:
        return round(st["mean"])
    return NO_REF


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "field"
    latest = {}
    with open(SOURCE) as captures:
        for line in captures:
            record = json.loads(line)
            latest[record["pose"]] = record
    missing = [p for p in POSES if p not in latest]
    if missing:
        sys.exit("no capture yet for: " + ", ".join(missing))
    out = ROOT / "calibration"
    out.mkdir(exist_ok=True)
    shutil.copyfile(SOURCE, out / (name + ".jsonl"))
    targets = {}
    for pose in POSES:
        c = latest[pose]
        targets[pose] = {"label": c["label"], "captured": c["started"], "note": c["note"],
                         "samples": c["samples_n"], "warnings": c["warnings"],
                         "ref": dict(zip(REF_KEYS, (steady(c, k) for k in REF_KEYS))),
                         "color_rg_ratio": c["stats"].get("color_rg_ratio", {}).get("mean")}
    summary = {"ref_order": list(REF_KEYS), "no_ref": NO_REF,
               "raw_captures": name + ".jsonl", "targets": targets}
    with open(out / (name + ".json"), "w") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    for pose in POSES:
        print("%-13s %s" % (pose, " ".join("%5s" % ("-" if v == NO_REF else v)
                                            for v in targets[pose]["ref"].values())))
    print("wrote", out / (name + ".json"), "and", out / (name + ".jsonl"))


if __name__ == "__main__":
    main()
