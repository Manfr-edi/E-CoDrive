"""Apply a rigid 2D transform to the nodes of a Lanelet2 OSM map."""

import argparse
import math
import xml.etree.ElementTree as ET

from lanelet2.core import BasicPoint3d, GPSPoint
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("destination")
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    parser.add_argument("--rotation-deg", type=float, required=True)
    parser.add_argument("--translation-x", type=float, required=True)
    parser.add_argument("--translation-y", type=float, required=True)
    args = parser.parse_args()

    projector = UtmProjector(
        Origin(args.origin_lat, args.origin_lon),
        True,
        False,
    )
    angle = math.radians(args.rotation_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)

    tree = ET.parse(args.source)
    root = tree.getroot()
    transformed = 0
    for node in root.findall("node"):
        gps = GPSPoint(
            float(node.get("lat")),
            float(node.get("lon")),
            0.0,
        )
        point = projector.forward(gps)
        aligned_x = cosine * point.x - sine * point.y + args.translation_x
        aligned_y = sine * point.x + cosine * point.y + args.translation_y
        aligned_gps = projector.reverse(BasicPoint3d(aligned_x, aligned_y, 0.0))
        node.set("lat", f"{aligned_gps.lat:.15f}")
        node.set("lon", f"{aligned_gps.lon:.15f}")

        # The old MGRS value describes the unaligned coordinate and is only
        # metadata; Lanelet2 does not need it for loading or routing.
        for tag in list(node.findall("tag")):
            if tag.get("k") == "mgrs_code":
                node.remove(tag)
        transformed += 1

    tree.write(args.destination, encoding="UTF-8", xml_declaration=True)
    print(f"transformed_nodes={transformed}")


if __name__ == "__main__":
    main()
