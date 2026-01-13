#!/usr/bin/env python3
"""
Test 3: KML Path Planning Test

Tests:
- Load input KML polygon
- Generate lawnmower survey waypoints
- Output KML file with plotted path
- Display path visualization
"""

import sys
import os
import argparse

sys.path.insert(0, '..')
from config import MissionConfig
from kml_processor import KMLProcessor


def create_output_kml(waypoints, output_path: str, polygon_coords: list = None):
    """
    Create KML file with survey path and optional polygon outline.
    
    Args:
        waypoints: List of Waypoint objects
        output_path: Output KML file path
        polygon_coords: Original polygon coordinates (lon, lat) pairs
    """
    kml_header = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Survey Flight Path</name>
    <description>Generated lawnmower survey path</description>
    
    <!-- Path Style -->
    <Style id="pathStyle">
      <LineStyle>
        <color>ff0000ff</color>
        <width>3</width>
      </LineStyle>
    </Style>
    
    <!-- Waypoint Style -->
    <Style id="waypointStyle">
      <IconStyle>
        <scale>0.6</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/paddle/ylw-circle.png</href>
        </Icon>
      </IconStyle>
      <LabelStyle>
        <scale>0.7</scale>
      </LabelStyle>
    </Style>
    
    <!-- Polygon Style -->
    <Style id="polygonStyle">
      <LineStyle>
        <color>ff00ff00</color>
        <width>2</width>
      </LineStyle>
      <PolyStyle>
        <color>2000ff00</color>
      </PolyStyle>
    </Style>
'''
    
    kml_footer = '''  </Document>
</kml>
'''
    
    with open(output_path, 'w') as f:
        f.write(kml_header)
        
        # Add original polygon if provided
        if polygon_coords:
            f.write('''
    <!-- Survey Area Polygon -->
    <Placemark>
      <name>Survey Area</name>
      <styleUrl>#polygonStyle</styleUrl>
      <Polygon>
        <tessellate>1</tessellate>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
''')
            for lon, lat in polygon_coords:
                f.write(f"              {lon},{lat},0\n")
            f.write('''            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
''')
        
        # Add flight path as LineString
        f.write('''
    <!-- Flight Path -->
    <Placemark>
      <name>Flight Path</name>
      <description>Lawnmower survey pattern</description>
      <styleUrl>#pathStyle</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>
''')
        for wp in waypoints:
            f.write(f"          {wp.lon},{wp.lat},{wp.alt}\n")
        
        f.write('''        </coordinates>
      </LineString>
    </Placemark>
''')
        
        # Add individual waypoints
        f.write('''
    <!-- Waypoints Folder -->
    <Folder>
      <name>Waypoints</name>
''')
        for wp in waypoints:
            f.write(f'''      <Placemark>
        <name>WP{wp.index}</name>
        <description>Altitude: {wp.alt}m</description>
        <styleUrl>#waypointStyle</styleUrl>
        <Point>
          <altitudeMode>relativeToGround</altitudeMode>
          <coordinates>{wp.lon},{wp.lat},{wp.alt}</coordinates>
        </Point>
      </Placemark>
''')
        
        f.write('''    </Folder>
''')
        
        f.write(kml_footer)


def main():
    parser = argparse.ArgumentParser(
        description='KML Path Planning Test - Generate survey waypoints from KML polygon',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
    python test_kml_path.py --input ../config/survey_area.kml
    python test_kml_path.py --input my_area.kml --output flight_path.kml
    python test_kml_path.py --input area.kml --spacing 20 --interval 25
'''
    )
    
    parser.add_argument('--input', '-i', required=True,
                       help='Input KML file with survey polygon')
    parser.add_argument('--output', '-o',
                       help='Output KML file (default: <input>_path.kml)')
    parser.add_argument('--spacing', type=float, default=15.0,
                       help='Sweep line spacing in meters (default: 15)')
    parser.add_argument('--interval', type=float, default=20.0,
                       help='Waypoint interval in meters (default: 20)')
    parser.add_argument('--altitude', type=float, default=10.0,
                       help='Flight altitude in meters (default: 10)')
    
    args = parser.parse_args()
    
    # Validate input file
    if not os.path.exists(args.input):
        print(f"ERROR: Input file not found: {args.input}")
        return 1
    
    # Generate output filename if not provided
    if args.output:
        output_path = args.output
    else:
        base = os.path.splitext(args.input)[0]
        output_path = f"{base}_path.kml"
    
    print("="*60)
    print("KML PATH PLANNING TEST")
    print("="*60)
    print(f"\nInput:    {args.input}")
    print(f"Output:   {output_path}")
    print(f"Spacing:  {args.spacing}m")
    print(f"Interval: {args.interval}m")
    print(f"Altitude: {args.altitude}m")
    print()
    
    # Create processor
    config = {
        'sweep_spacing': args.spacing,
        'waypoint_interval': args.interval,
        'altitude': args.altitude
    }
    
    processor = KMLProcessor(config)
    
    # Load KML
    print("[1] Loading KML polygon...")
    if not processor.load(args.input):
        print("    FAILED: Could not load KML file")
        return 1
    
    print(f"    PASSED: Loaded '{processor.polygon_name}'")
    print(f"    Points: {len(processor.polygon_coords)}")
    
    # Get polygon info
    center = processor.get_polygon_center()
    if center:
        print(f"    Center: ({center[0]:.6f}, {center[1]:.6f})")
    
    area = processor.get_polygon_area()
    if area > 0:
        print(f"    Area:   {area:.0f} m2 ({area/10000:.2f} hectares)")
    
    # Generate waypoints
    print("\n[2] Generating survey waypoints...")
    waypoints = processor.generate_waypoints(
        spacing=args.spacing,
        interval=args.interval,
        altitude=args.altitude
    )
    
    if not waypoints:
        print("    FAILED: No waypoints generated")
        return 1
    
    print(f"    PASSED: Generated {len(waypoints)} waypoints")
    
    # Calculate path length
    total_distance = 0.0
    for i in range(1, len(waypoints)):
        lat1, lon1 = waypoints[i-1].lat, waypoints[i-1].lon
        lat2, lon2 = waypoints[i].lat, waypoints[i].lon
        
        # Simple distance approximation
        from math import radians, cos, sin, sqrt, atan2
        R = 6371000
        phi1, phi2 = radians(lat1), radians(lat2)
        dphi = radians(lat2 - lat1)
        dlam = radians(lon2 - lon1)
        a = sin(dphi/2)**2 + cos(phi1)*cos(phi2)*sin(dlam/2)**2
        total_distance += R * 2 * atan2(sqrt(a), sqrt(1-a))
    
    print(f"    Path length: {total_distance:.0f}m ({total_distance/1000:.2f}km)")
    
    # Estimate flight time (at 5 m/s)
    speed = 5.0
    flight_time = total_distance / speed
    print(f"    Est. flight time: {flight_time/60:.1f} min (at {speed} m/s)")
    
    # Create output KML
    print(f"\n[3] Creating output KML...")
    create_output_kml(waypoints, output_path, processor.polygon_coords)
    print(f"    PASSED: Saved to {output_path}")
    
    # Display first and last waypoints
    print("\n[4] Waypoint summary:")
    print("    First waypoints:")
    for wp in waypoints[:3]:
        print(f"      WP{wp.index}: ({wp.lat:.6f}, {wp.lon:.6f}, {wp.alt}m)")
    
    print("    ...")
    print("    Last waypoints:")
    for wp in waypoints[-3:]:
        print(f"      WP{wp.index}: ({wp.lat:.6f}, {wp.lon:.6f}, {wp.alt}m)")
    
    print("\n" + "="*60)
    print("TEST COMPLETED SUCCESSFULLY")
    print("="*60)
    print(f"\nOutput KML: {output_path}")
    print("Open in Google Earth to visualize the flight path.")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
