#!/usr/bin/env python3
"""Print the axis-aligned bounds of a binary PCD. Temporary inspection tool."""

import struct
import sys
from pathlib import Path


def _header_fields(header: str) -> dict[str, list[str]]:
    fields: dict[str, list[str]] = {}
    for line in header.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            fields[parts[0]] = parts[1:]
    return fields


def bounds(path: Path) -> tuple[int, list[float], list[float]]:
    """Return point count and the min/max of x, y, z."""
    data = path.read_bytes()
    marker = b"DATA binary\n"
    index = data.find(marker)
    if index < 0:
        raise ValueError(f"{path} is not an uncompressed binary PCD")

    fields = _header_fields(data[:index].decode("ascii"))
    if fields.get("FIELDS", [])[:3] != ["x", "y", "z"]:
        raise ValueError("PCD fields must start with x y z")
    if fields.get("SIZE", [])[:3] != ["4", "4", "4"] or fields.get("TYPE", [])[:3] != [
        "F",
        "F",
        "F",
    ]:
        raise ValueError("only float32 x y z is supported")

    point_size = sum(int(value) for value in fields["SIZE"])
    count = int(fields["POINTS"][0])
    body = data[index + len(marker) :]
    if len(body) < count * point_size:
        raise ValueError("PCD body is shorter than POINTS")

    minimum = [float("inf")] * 3
    maximum = [float("-inf")] * 3
    for point_index in range(count):
        xyz = struct.unpack_from("<fff", body, point_index * point_size)
        for axis, value in enumerate(xyz):
            if value < minimum[axis]:
                minimum[axis] = value
            if value > maximum[axis]:
                maximum[axis] = value
    return count, minimum, maximum


def main() -> None:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "obs1_filtered.pcd")
    count, minimum, maximum = bounds(path)
    names = ("x", "y", "z")
    print(f"{path} ({count} points)")
    for axis, name in enumerate(names):
        span = maximum[axis] - minimum[axis]
        print(
            f"{name} [{minimum[axis]:.4f}, {maximum[axis]:.4f}]  span {span:.4f}"
        )


if __name__ == "__main__":
    main()
