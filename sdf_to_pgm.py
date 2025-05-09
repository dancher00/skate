#!/usr/bin/env python3

import argparse
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw
import os

def parse_arguments():
    parser = argparse.ArgumentParser(description='Convert SDF world file to PGM map')
    parser.add_argument('sdf_file', help='Path to the SDF world file')
    parser.add_argument('--resolution', type=float, default=0.05, 
                        help='Map resolution in meters/pixel (default: 0.05)')
    parser.add_argument('--output_dir', default='.', 
                        help='Directory to save the output files (default: current directory)')
    parser.add_argument('--map_name', default='map', 
                        help='Name for the output files (default: map)')
    parser.add_argument('--square', action='store_true', 
                        help='Force square map even if world is rectangular')
    return parser.parse_args()

def parse_sdf_world(sdf_file):
    """Parse SDF world file to extract world size and obstacles"""
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    world = root.find('.//world')
    
    world_size = None
    obstacles = []
    
    # Find world boundaries based on walls
    walls = []
    north_wall = None
    south_wall = None
    east_wall = None
    west_wall = None
    
    # Extract all wall information
    for model in world.findall('.//model'):
        model_name = model.get('name', '')
        if 'wall' in model_name:
            link = model.find('.//link')
            if link is None:
                continue
                
            pose_elem = link.find('.//pose')
            if pose_elem is None:
                continue
                
            pose_text = pose_elem.text
            if pose_text:
                pose = [float(val) for val in pose_text.split()]
            else:
                continue
                
            box_elem = link.find('.//geometry/box/size')
            if box_elem is None:
                continue
                
            size_text = box_elem.text
            if size_text:
                size = [float(val) for val in size_text.split()]
            else:
                continue
            
            wall_info = {
                'name': model_name,
                'pose': pose,
                'size': size
            }
            
            walls.append(wall_info)
            
            if 'north_wall' in model_name:
                north_wall = wall_info
            elif 'south_wall' in model_name:
                south_wall = wall_info
            elif 'east_wall' in model_name:
                east_wall = wall_info
            elif 'west_wall' in model_name:
                west_wall = wall_info
    
    # Calculate world dimensions based on wall positions
    if north_wall and south_wall:
        # Calculate height based on wall positions (distance between walls + half their thickness)
        world_height = abs(north_wall['pose'][1] - south_wall['pose'][1])
        print(f"North wall at y={north_wall['pose'][1]}, South wall at y={south_wall['pose'][1]}")
        print(f"Calculated world height: {world_height}m")
    else:
        world_height = 20.0
        print(f"Could not detect north/south walls, using default world height: {world_height}m")
    
    if east_wall and west_wall:
        # Calculate width based on wall positions (distance between walls + half their thickness)
        world_width = abs(east_wall['pose'][0] - west_wall['pose'][0])
        print(f"East wall at x={east_wall['pose'][0]}, West wall at x={west_wall['pose'][0]}")
        print(f"Calculated world width: {world_width}m")
    else:
        world_width = 20.0
        print(f"Could not detect east/west walls, using default world width: {world_width}m")
    
    # Set final world size
    world_size = (world_width, world_height)
    print(f"Determined world size: {world_size}m")
    
    # Get obstacles (non-wall objects)
    for model in world.findall('.//model'):
        model_name = model.get('name', '')
        if 'wall' in model_name or 'origin_marker' in model_name:
            continue
            
        for link in model.findall('.//link'):
            pose_elem = link.find('.//pose')
            if pose_elem is None:
                continue
                
            pose_text = pose_elem.text
            if pose_text:
                pose = [float(val) for val in pose_text.split()]
            else:
                continue
            
            # Check for box
            box_elem = link.find('.//geometry/box/size')
            if box_elem is not None:
                size_text = box_elem.text
                if size_text:
                    size = [float(val) for val in size_text.split()]
                    obstacles.append({
                        'type': 'box',
                        'pose': pose,
                        'size': size
                    })
                continue
                
            # Check for cylinder
            cylinder_elem = link.find('.//geometry/cylinder')
            if cylinder_elem is not None:
                radius_elem = cylinder_elem.find('.//radius')
                length_elem = cylinder_elem.find('.//length')
                
                if radius_elem is not None and length_elem is not None:
                    radius = float(radius_elem.text)
                    length = float(length_elem.text)
                    obstacles.append({
                        'type': 'cylinder',
                        'pose': pose,
                        'radius': radius,
                        'length': length
                    })
                continue
                
            # Check for sphere
            sphere_elem = link.find('.//geometry/sphere')
            if sphere_elem is not None:
                radius_elem = sphere_elem.find('.//radius')
                
                if radius_elem is not None:
                    radius = float(radius_elem.text)
                    obstacles.append({
                        'type': 'sphere',
                        'pose': pose,
                        'radius': radius
                    })
    
    print(f"Found {len(obstacles)} obstacles")
    return world_size, obstacles

def create_pgm_map(world_size, obstacles, resolution, output_dir, map_name, force_square=False):
    """Create a PGM map file and corresponding YAML configuration"""
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get world dimensions
    width_meters, height_meters = world_size
    
    # Make square if requested
    if force_square:
        max_dim = max(width_meters, height_meters)
        width_meters = height_meters = max_dim
        print(f"Forcing square map: {width_meters}m × {height_meters}m")
    
    # Calculate map dimensions in pixels
    width_pixels = int(width_meters / resolution)
    height_pixels = int(height_meters / resolution)
    
    # Create a blank image (white background = free space)
    img = Image.new('L', (width_pixels, height_pixels), color=255)
    draw = ImageDraw.Draw(img)
    
    # Function to convert world coordinates to image coordinates
    def world_to_img(x, y):
        # In world coordinates, (0,0) is at the center
        # In image coordinates, (0,0) is at the top-left
        img_x = int((x + width_meters / 2) / resolution)
        img_y = int((height_meters / 2 - y) / resolution)
        return img_x, img_y
    
    # Draw obstacles (black = occupied)
    for obstacle in obstacles:
        if obstacle['type'] == 'box':
            pose = obstacle['pose']
            size = obstacle['size']
            
            # Calculate box corners
            half_width = size[0] / 2
            half_depth = size[1] / 2
            
            # Apply rotation if any
            yaw = pose[5] if len(pose) > 5 else 0
            corners = []
            for dx, dy in [(-half_width, -half_depth), (half_width, -half_depth), 
                           (half_width, half_depth), (-half_width, half_depth)]:
                # Rotate point
                rotated_dx = dx * np.cos(yaw) - dy * np.sin(yaw)
                rotated_dy = dx * np.sin(yaw) + dy * np.cos(yaw)
                
                # Get image coordinates
                img_x, img_y = world_to_img(pose[0] + rotated_dx, pose[1] + rotated_dy)
                corners.append((img_x, img_y))
            
            draw.polygon(corners, fill=0)
        
        elif obstacle['type'] == 'cylinder' or obstacle['type'] == 'sphere':
            pose = obstacle['pose']
            radius = obstacle['radius']
            
            # Calculate bounding box for circle
            center_x, center_y = world_to_img(pose[0], pose[1])
            radius_pixels = int(radius / resolution)
            
            # Draw circle
            draw.ellipse(
                (center_x - radius_pixels, center_y - radius_pixels,
                 center_x + radius_pixels, center_y + radius_pixels),
                fill=0
            )
    
    # Draw world boundaries (walls)
    wall_thickness = int(0.2 / resolution)  # Typical wall thickness is 0.2m
    
    # Draw north wall (top)
    draw.rectangle([(0, 0), (width_pixels, wall_thickness)], fill=0)
    
    # Draw south wall (bottom)
    draw.rectangle([(0, height_pixels-wall_thickness), (width_pixels, height_pixels)], fill=0)
    
    # Draw east wall (right)
    draw.rectangle([(width_pixels-wall_thickness, 0), (width_pixels, height_pixels)], fill=0)
    
    # Draw west wall (left)
    draw.rectangle([(0, 0), (wall_thickness, height_pixels)], fill=0)
    
    # Save PGM file manually in binary format
    pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
    
    # Convert PIL Image to numpy array
    img_array = np.array(img)
    
    # Write PGM file in P5 (binary) format
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n')  # Binary PGM format
        f.write(f'{width_pixels} {height_pixels}\n'.encode())
        f.write(b'255\n')  # Max value
        f.write(img_array.astype(np.uint8).tobytes())
    
    print(f"Saved PGM map with dimensions: {width_pixels}x{height_pixels} pixels")
    
    # Calculate origin for YAML file (bottom-left corner in world coordinates)
    origin_x = -width_meters / 2
    origin_y = -height_meters / 2
    
    # Create YAML configuration file
    yaml_path = os.path.join(output_dir, f"{map_name}.yaml")
    with open(yaml_path, 'w') as yaml_file:
        yaml_file.write(f"image: {map_name}.pgm\n")
        yaml_file.write(f"resolution: {resolution}\n")
        yaml_file.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
        yaml_file.write("occupied_thresh: 0.65\n")
        yaml_file.write("free_thresh: 0.196\n")
        yaml_file.write("negate: 0\n")
    
    print(f"Map origin set to: [{origin_x}, {origin_y}, 0.0]")
    return pgm_path, yaml_path

def main():
    args = parse_arguments()
    
    print(f"Parsing SDF file: {args.sdf_file}")
    world_size, obstacles = parse_sdf_world(args.sdf_file)
    
    if world_size is None:
        # Default size if we couldn't determine from walls
        world_size = (20.0, 20.0)
        print(f"Warning: Could not determine world size from walls, using default: {world_size}")
    
    print(f"Creating map with resolution: {args.resolution}m/px")
    pgm_path, yaml_path = create_pgm_map(
        world_size, obstacles, args.resolution, args.output_dir, args.map_name, args.square
    )
    
    print(f"Map created successfully!")
    print(f"PGM file: {pgm_path}")
    print(f"YAML file: {yaml_path}")
    print(f"Use these files with the nav2_map_server node:")
    print(f"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml_path}")

if __name__ == "__main__":
    main()