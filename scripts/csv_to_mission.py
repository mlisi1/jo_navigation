#!/usr/bin/env python3
"""
Emlid CSV -> Mission YAML Converter
------------------------------------
Converts a points CSV exported from Emlid Reach (Reach RS/RS2/RS4, via
Emlid Flow / Emlid Studio) into the mission YAML format consumed by
send_gps_waypoints.py.

Expected CSV columns (standard Emlid "points" export):
  Name, Description, Longitude, Latitude, Elevation, Ellipsoidal height, ...
Only Name, Description, Longitude, Latitude, Elevation and Ellipsoidal
height are used; every other column (RMS, solution status, base station
info, etc.) is ignored.

By default, rows become points in the same order they appear in the CSV,
and the generated 'mission' section visits them in that same order. Edit
the output file afterwards to reorder, drop, or repeat waypoints.

Usage:
  csv_to_mission.py points.csv
  csv_to_mission.py points.csv -o mission.yaml
  csv_to_mission.py points.csv --altitude elevation
  csv_to_mission.py points.csv --prefix spot
"""

import argparse
import csv
import re
import sys
from pathlib import Path


def slugify(text: str) -> str:
    """Turn arbitrary text into a safe, YAML-key-friendly label."""
    text = text.strip().lower()
    text = re.sub(r'[^a-z0-9]+', '_', text)
    return text.strip('_')


def make_label(row: dict, index: int, prefix: str, used: set) -> str:
    """Prefer the CSV 'Description', fall back to '<prefix><Name>'."""
    description = (row.get('Description') or '').strip()
    name        = (row.get('Name') or '').strip()

    if description:
        label = slugify(description)
    elif name:
        label = f'{prefix}{slugify(name)}'
    else:
        label = f'{prefix}{index}'

    if not label:
        label = f'{prefix}{index}'

    # Guarantee uniqueness in case of repeated descriptions/names.
    base = label
    n = 2
    while label in used:
        label = f'{base}_{n}'
        n += 1

    return label


def load_points(csv_path: Path, prefix: str, altitude: str) -> list:
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)

        required = {'Latitude', 'Longitude'}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f"CSV is missing required column(s): {', '.join(sorted(missing))}"
            )

        alt_column = {
            'ellipsoidal': 'Ellipsoidal height',
            'elevation':   'Elevation',
        }.get(altitude)

        if alt_column and alt_column not in reader.fieldnames:
            raise ValueError(f"CSV is missing the '{alt_column}' column.")

        points = []
        used_labels = set()
        for i, row in enumerate(reader, start=1):
            lat_str = (row.get('Latitude') or '').strip()
            lon_str = (row.get('Longitude') or '').strip()
            if not lat_str or not lon_str:
                print(f'Skipping row {i}: missing latitude/longitude.', file=sys.stderr)
                continue

            label = make_label(row, i, prefix, used_labels)
            used_labels.add(label)

            alt = 0.0
            if alt_column:
                alt_str = (row.get(alt_column) or '').strip()
                alt = float(alt_str) if alt_str else 0.0

            points.append({
                'label': label,
                'lat':   float(lat_str),
                'lon':   float(lon_str),
                'alt':   alt,
            })

        return points


def write_mission(points: list, out_path: Path, altitude: str, source_name: str):
    lines = [
        '# GPS Waypoint Mission File',
        '# ─────────────────────────',
        f'# Auto-generated from "{source_name}" by csv_to_mission.py.',
        f'# Altitude source: {altitude}',
        '# Review the point order below and edit the "mission" list to match',
        '# the desired visiting order before running the robot.',
        '',
        'points:',
    ]

    for pt in points:
        lines.append(f'  {pt["label"]}:')
        lines.append(f'    lat: {pt["lat"]:.8f}')
        lines.append(f'    lon: {pt["lon"]:.8f}')
        if altitude != 'none':
            lines.append(f'    alt: {pt["alt"]:.3f}')
        lines.append('')

    lines.append('mission:')
    for pt in points:
        lines.append(f'  - {pt["label"]}')
    lines.append('')

    out_path.write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(
        description='Convert an Emlid Reach points CSV into a mission YAML file.'
    )
    parser.add_argument('csv_file', type=Path, help='Path to the Emlid points CSV export.')
    parser.add_argument(
        '-o', '--output', type=Path, default=None,
        help='Output YAML path (default: same name as input, .yaml extension, same folder).'
    )
    parser.add_argument(
        '--prefix', default='wp',
        help='Label prefix used when a point has no Description (default: "wp", so Name "3" -> "wp3").'
    )
    parser.add_argument(
        '--altitude', choices=['ellipsoidal', 'elevation', 'none'], default='ellipsoidal',
        help=(
            "Which CSV column to use for 'alt': 'ellipsoidal' (WGS84 ellipsoidal "
            "height, matches raw GPS/NavSatFix altitude — default), 'elevation' "
            "(orthometric/MSL height), or 'none' to omit alt (defaults to 0.0)."
        )
    )
    args = parser.parse_args()

    if not args.csv_file.is_file():
        parser.error(f'CSV file not found: {args.csv_file}')

    output_path = args.output or args.csv_file.with_suffix('.yaml')

    points = load_points(args.csv_file, args.prefix, args.altitude)
    if not points:
        parser.error('No valid points found in CSV.')

    write_mission(points, output_path, args.altitude, args.csv_file.name)

    print(f'Wrote {len(points)} waypoint(s) to {output_path}')
    print('Points: ' + ', '.join(pt['label'] for pt in points))


if __name__ == '__main__':
    main()
