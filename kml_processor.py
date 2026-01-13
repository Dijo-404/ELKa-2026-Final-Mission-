#!/usr/bin/env python3
"""
KML Processor - Parse KML polygons and generate survey waypoints

Generates lawnmower (sweep) pattern for area coverage surveys.
Uses shapely and pyproj for accurate geodetic calculations.
"""

import logging
import math
import xml.etree.ElementTree as ET
from typing import List, Tuple, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# Try to import geodetic libraries
try:
    from shapely.geometry import Polygon, LineString
    from shapely.ops import transform as shapely_transform
    import pyproj
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    logger.warning("shapely/pyproj not installed - using simple waypoint generation")


@dataclass
class Waypoint:
    """Survey waypoint with GPS coordinates."""
    lat: float
    lon: float
    alt: float
    index: int = 0
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """Return as (lat, lon, alt) tuple."""
        return (self.lat, self.lon, self.alt)
    
    def __str__(self) -> str:
        return f"WP{self.index}: ({self.lat:.6f}, {self.lon:.6f}, {self.alt:.1f}m)"


def get_utm_crs(lat: float, lon: float) -> str:
    """
    Get appropriate UTM CRS string for a given latitude/longitude.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        
    Returns:
        EPSG code string for UTM zone
    """
    zone_number = int((lon + 180) / 6) + 1
    if lat >= 0:
        return f"EPSG:{32600 + zone_number}"
    else:
        return f"EPSG:{32700 + zone_number}"


class KMLProcessor:
    """
    Parse KML polygon and generate survey waypoints.
    
    Supports lawnmower/sweep pattern generation for area surveys.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize KML processor.
        
        Args:
            config: Optional configuration dictionary with:
                - sweep_spacing: Distance between sweep lines (meters)
                - waypoint_interval: Distance between waypoints (meters)
                - altitude: Default flight altitude (meters)
        """
        self.config = config or {}
        self.polygon_coords: List[Tuple[float, float]] = []  # (lon, lat) pairs
        self.waypoints: List[Waypoint] = []
        self.polygon_name = ""
        
        # Default settings
        self.default_spacing = self.config.get('sweep_spacing', 15.0)
        self.default_interval = self.config.get('waypoint_interval', 20.0)
        self.default_altitude = self.config.get('altitude', 25.0)
    
    def load(self, filepath: str) -> bool:
        """
        Load and parse KML file.
        
        Args:
            filepath: Path to KML file
            
        Returns:
            True if polygon loaded successfully
        """
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
            
            # Handle KML namespace
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            
            # Find coordinates element
            coords_elem = root.find('.//kml:coordinates', ns)
            if coords_elem is None:
                # Try without namespace
                coords_elem = root.find('.//coordinates')
            
            if coords_elem is None:
                logger.error("No coordinates found in KML")
                return False
            
            # Parse coordinates
            coords_text = coords_elem.text.strip()
            self.polygon_coords = []
            
            for coord_str in coords_text.split():
                parts = coord_str.split(',')
                if len(parts) >= 2:
                    lon = float(parts[0])
                    lat = float(parts[1])
                    self.polygon_coords.append((lon, lat))
            
            # Get polygon name
            name_elem = root.find('.//kml:name', ns) or root.find('.//name')
            self.polygon_name = name_elem.text if name_elem is not None else "Survey Area"
            
            logger.info(f"Loaded KML: {self.polygon_name} with {len(self.polygon_coords)} points")
            return len(self.polygon_coords) >= 3
            
        except Exception as e:
            logger.error(f"Failed to load KML: {e}")
            return False
    
    def generate_waypoints(self, 
                          spacing: float = None,
                          interval: float = None,
                          altitude: float = None) -> List[Waypoint]:
        """
        Generate lawnmower sweep pattern waypoints within the polygon.
        
        Args:
            spacing: Distance between sweep lines (meters)
            interval: Distance between waypoints along lines (meters)
            altitude: Flight altitude (meters)
            
        Returns:
            List of Waypoint objects
        """
        spacing = spacing or self.default_spacing
        interval = interval or self.default_interval
        altitude = altitude or self.default_altitude
        
        if len(self.polygon_coords) < 3:
            logger.error("No polygon loaded")
            return []
        
        self.waypoints = []
        
        if SHAPELY_AVAILABLE:
            self._generate_with_shapely(spacing, interval, altitude)
        else:
            self._generate_simple(altitude)
        
        logger.info(f"Generated {len(self.waypoints)} waypoints")
        return self.waypoints
    
    def _generate_with_shapely(self, spacing: float, interval: float, altitude: float):
        """
        Generate waypoints using shapely for accurate geodetic calculations.
        
        Uses UTM projection for meter-based calculations.
        """
        # Create polygon from coordinates (lon, lat order for shapely)
        poly_lonlat = Polygon(self.polygon_coords)
        
        if not poly_lonlat.is_valid or poly_lonlat.is_empty:
            logger.error("Invalid polygon geometry")
            self._generate_simple(altitude)
            return
        
        # Get centroid for UTM zone calculation
        centroid = poly_lonlat.centroid
        utm_crs = get_utm_crs(centroid.y, centroid.x)
        
        # Create coordinate transformers
        project_to_utm = pyproj.Transformer.from_crs(
            "EPSG:4326", utm_crs, always_xy=True
        ).transform
        project_to_latlon = pyproj.Transformer.from_crs(
            utm_crs, "EPSG:4326", always_xy=True
        ).transform
        
        # Transform polygon to UTM
        poly_utm = shapely_transform(project_to_utm, poly_lonlat)
        
        # Get bounding box in UTM
        minx, miny, maxx, maxy = poly_utm.bounds
        
        # Generate sweep lines
        y = miny
        lines = []
        
        while y <= maxy:
            line = LineString([(minx - 10, y), (maxx + 10, y)])
            segment = line.intersection(poly_utm)
            
            if not segment.is_empty:
                if segment.geom_type == 'LineString':
                    lines.append(segment)
                elif segment.geom_type == 'MultiLineString':
                    for seg in segment.geoms:
                        lines.append(seg)
            
            y += spacing
        
        # Sort lines by Y coordinate
        lines = sorted(lines, key=lambda l: l.centroid.y)
        
        # Generate waypoints along lines (alternating direction)
        idx = 0
        reverse = False
        
        for line in lines:
            length = line.length
            num_points = max(int(length // interval) + 1, 2)
            segment_points = []
            
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0
                pt = line.interpolate(t * length)
                
                # Transform back to lat/lon
                lon, lat = project_to_latlon(pt.x, pt.y)
                segment_points.append(Waypoint(
                    lat=lat, lon=lon, alt=altitude, index=idx
                ))
                idx += 1
            
            # Alternate direction for efficient path
            if reverse:
                segment_points.reverse()
                # Update indices
                for i, wp in enumerate(segment_points):
                    wp.index = idx - len(segment_points) + i
            
            self.waypoints.extend(segment_points)
            reverse = not reverse
    
    def _generate_simple(self, altitude: float):
        """
        Simple waypoint generation (polygon corners only).
        
        Fallback when shapely is not available.
        """
        logger.warning("Using simple waypoint generation (polygon corners only)")
        for idx, (lon, lat) in enumerate(self.polygon_coords):
            self.waypoints.append(Waypoint(
                lat=lat, lon=lon, alt=altitude, index=idx
            ))
    
    def get_waypoints(self) -> List[Tuple[float, float, float]]:
        """Get waypoints as list of (lat, lon, alt) tuples."""
        return [wp.to_tuple() for wp in self.waypoints]
    
    def get_polygon_center(self) -> Optional[Tuple[float, float]]:
        """Get center of polygon as (lat, lon)."""
        if not self.polygon_coords:
            return None
        
        lons = [c[0] for c in self.polygon_coords]
        lats = [c[1] for c in self.polygon_coords]
        
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
        
        return (center_lat, center_lon)
    
    def get_waypoint_count(self) -> int:
        """Get number of generated waypoints."""
        return len(self.waypoints)
    
    def get_polygon_area(self) -> float:
        """
        Get approximate area of polygon in square meters.
        
        Returns:
            Area in square meters, or 0 if calculation fails
        """
        if not SHAPELY_AVAILABLE or len(self.polygon_coords) < 3:
            return 0.0
        
        try:
            poly_lonlat = Polygon(self.polygon_coords)
            centroid = poly_lonlat.centroid
            utm_crs = get_utm_crs(centroid.y, centroid.x)
            
            project_to_utm = pyproj.Transformer.from_crs(
                "EPSG:4326", utm_crs, always_xy=True
            ).transform
            
            poly_utm = shapely_transform(project_to_utm, poly_lonlat)
            return poly_utm.area
        except:
            return 0.0
    
    def write_waypoints_kml(self, output_path: str) -> bool:
        """
        Write generated waypoints to a KML file.
        
        Args:
            output_path: Output KML file path
            
        Returns:
            True if successful
        """
        if not self.waypoints:
            logger.error("No waypoints to write")
            return False
        
        try:
            kml_ns = "http://www.opengis.net/kml/2.2"
            ET.register_namespace('', kml_ns)
            
            kml_elem = ET.Element(f"{{{kml_ns}}}kml")
            doc_elem = ET.SubElement(kml_elem, "Document")
            
            placemark = ET.SubElement(doc_elem, "Placemark")
            name = ET.SubElement(placemark, "name")
            name.text = "Generated Flight Path"
            
            ls = ET.SubElement(placemark, "LineString")
            tessellate = ET.SubElement(ls, "tessellate")
            tessellate.text = "1"
            altitude_mode = ET.SubElement(ls, "altitudeMode")
            altitude_mode.text = "relativeToGround"
            
            coords_elem = ET.SubElement(ls, "coordinates")
            coords_text = " ".join(
                f"{wp.lon},{wp.lat},{wp.alt}" for wp in self.waypoints
            )
            coords_elem.text = coords_text
            
            tree = ET.ElementTree(kml_elem)
            tree.write(output_path, encoding="utf-8", xml_declaration=True)
            
            logger.info(f"Waypoints written to {output_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to write KML: {e}")
            return False


# =============================================================================
# TEST CODE
# =============================================================================
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )
    
    print("KML Processor Test")
    print("=" * 50)
    print(f"Shapely available: {SHAPELY_AVAILABLE}")
    
    # Create sample KML for testing
    sample_kml = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Test Survey Area</name>
    <Placemark>
      <name>Survey Zone</name>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              77.590,12.970,0
              77.592,12.970,0
              77.592,12.972,0
              77.590,12.972,0
              77.590,12.970,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
"""
    
    # Write sample KML
    import tempfile
    import os
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.kml', delete=False) as f:
        f.write(sample_kml)
        temp_kml = f.name
    
    try:
        processor = KMLProcessor({
            'sweep_spacing': 20.0,
            'waypoint_interval': 25.0,
            'altitude': 25.0
        })
        
        if processor.load(temp_kml):
            print(f"\nLoaded: {processor.polygon_name}")
            print(f"Polygon points: {len(processor.polygon_coords)}")
            
            center = processor.get_polygon_center()
            if center:
                print(f"Center: ({center[0]:.6f}, {center[1]:.6f})")
            
            area = processor.get_polygon_area()
            if area > 0:
                print(f"Area: {area:.1f} m² ({area/10000:.2f} hectares)")
            
            waypoints = processor.generate_waypoints()
            print(f"\nGenerated {len(waypoints)} waypoints:")
            
            for wp in waypoints[:5]:
                print(f"  {wp}")
            
            if len(waypoints) > 5:
                print(f"  ... and {len(waypoints) - 5} more")
        else:
            print("Failed to load KML")
    
    finally:
        os.unlink(temp_kml)
