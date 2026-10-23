"""Record Mega CSV telemetry on any computer. Never sends drive commands."""
import argparse
import csv
import sys

from telemetry_schema import FIELDS, parse_line


def record(stream, output):
    writer = csv.writer(output)
    writer.writerow(FIELDS)
    count = 0
    try:
        while True:
            raw = stream.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            data = parse_line(line)
            if data is None:
                continue
            writer.writerow([data[k] for k in FIELDS])
            output.flush()
            count += 1
    except KeyboardInterrupt:
        return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", required=True, help="New CSV file; existing files are never overwritten")
    args = parser.parse_args()
    try:
        import serial
        with open(args.output, "x", newline="") as output:
            with serial.Serial(args.port, args.baud, timeout=0.5) as stream:
                print("Recording. Opening USB may reset the Mega; start recording before ARM/START.")
                count = record(stream, output)
        print("Saved {} rows to {}".format(count, args.output))
        return 0
    except Exception as exc:
        print("Recording failed: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
