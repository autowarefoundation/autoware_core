#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import xml.etree.ElementTree as ET

import mgrs
from pyproj import Transformer

# Refer sample TOKYO lat/lon coordinates from
# https://github.com/autowarefoundation/autoware.universe/blob/248bba7f2a3cd6a8d0777350c6dce062e79f5967/map/autoware_map_projection_loader/README.md
GLOBAL_ORIGIN_LAT = 35.6762
GLOBAL_ORIGIN_LON = 139.6503


def local_xy_to_global_latlon(local_x, local_y):
    # Note:
    # EPSG:4326 is the WGS84 coordinate system widely used for GPS and mapping (latitude/longitude format).
    # EPSG:32654 is the UTM Zone 54N coordinate system based on WGS84, which is used for local maps in TOKYO.
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32654", always_xy=True)
    transformer_to_wgs84 = Transformer.from_crs("EPSG:32654", "EPSG:4326", always_xy=True)
    new_origin_x, new_origin_y = transformer_to_utm.transform(GLOBAL_ORIGIN_LON, GLOBAL_ORIGIN_LAT)
    lon, lat = transformer_to_wgs84.transform(new_origin_x + local_x, new_origin_y + local_y)
    return lat, lon


# the coordinate of specified ID point on VMB
new_local_origin_x = 100.0
new_local_origin_y = 100.0


def update_osm_latlon(osm_file, output_file, origin_id):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    old_origin_local_xy = None

    for node in root.findall("node"):
        local_x_tag = node.find(".//tag[@k='local_x']")
        local_y_tag = node.find(".//tag[@k='local_y']")

        if node.attrib["id"] == str(origin_id):
            old_origin_local_xy = (float(local_x_tag.attrib["v"]), float(local_y_tag.attrib["v"]))
            break

    if old_origin_local_xy is None:
        print(f"could not find point of id {origin_id}")
        return

    (old_origin_x, old_origin_y) = old_origin_local_xy

    mgrs_code = mgrs.MGRS().toMGRS(GLOBAL_ORIGIN_LAT, GLOBAL_ORIGIN_LON)

    for node in root.findall("node"):
        local_x_tag = node.find(".//tag[@k='local_x']")
        local_y_tag = node.find(".//tag[@k='local_y']")

        local_x = float(local_x_tag.attrib["v"])
        local_y = float(local_y_tag.attrib["v"])
        rel_x = local_x - old_origin_x  # for origin, this is 0
        rel_y = local_y - old_origin_y  # for origin, this is 0

        new_local_x = new_local_origin_x + rel_x
        new_local_y = new_local_origin_y + rel_y

        # set lat, lon
        new_global_lat, new_global_lon = local_xy_to_global_latlon(new_local_x, new_local_y)
        node.set("lat", str(new_global_lat))
        node.set("lon", str(new_global_lon))

        # set local_x, local_y
        local_x_tag.set("v", str(new_local_x))
        local_y_tag.set("v", str(new_local_y))

        # set MGRS
        mgrs_tag = node.find(".//tag[@k='mgrs_code']")
        if mgrs_tag is not None:
            mgrs_tag.set("v", mgrs_code)

    tree.write(output_file, encoding="UTF-8", xml_declaration=True)
    print(f"Updated OSM file created: {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Update OSM file with new origin and adjusted coordinates."
    )
    parser.add_argument("input_osm", help="Path to the input OSM file")
    parser.add_argument("output_osm", help="Path to the output OSM file")
    parser.add_argument("origin_id", help="id of the point to reset as origin", type=int)
    args = parser.parse_args()

    update_osm_latlon(args.input_osm, args.output_osm, args.origin_id)
