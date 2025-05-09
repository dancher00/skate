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
    return parser.parse_args()

def parse_sdf_world(sdf_file):
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    world = root.find('.//world')
    
    world_size = None
    obstacles = []
    
    # Find world boundaries
    walls = []
    for model in world.findall('.//model'):
        model_name = model.get('name', '')
        if 'wall' in model_name:
            link = model.find('.//link')
            if link is None:
                continue
                
            pose_elem = link.find('.//pose')
            if pose_elem is None:
                continue
                
            pose = [float(val) for val in pose_elem.text.split()]
            
            box_elem = link.find('.//geometry/box/size')
            if box_elem is None:
                continue
                
            size = [float(val) for val in box_elem.text.split()]
            
            walls.append({
                'name': model_name,
                'pose': pose,
                'size': size
            })
            
            if 'north_wall' in model_name or 'south_wall' in model_name:
                world_size = (size[0], size[0] * 2) if world_size is None else world_size
    
    # Get obstacles
    for model in world.findall('.//model'):
        model_name = model.get('name', '')
        if 'wall' in model_name or 'origin_marker' in model_name:
            continue
            
        for link in model.findall('.//link'):
            pose_elem = link.find('.//pose')
            if pose_elem is None:
                continue
                
            pose = [float(val) for val in pose_elem.text.split()]
            
            # Check for box
            box_elem = link.find('.//geometry/box/size')
            if box_elem is not None:
                size = [float(val) for val in box_elem.text.split()]
                obstacles.append({
                    'type': 'box',
                    'pose': pose,
                    'size': size
                })
                continue
                
            # Check for cylinder
            cylinder_elem = link.find('.//geometry/cylinder')
            if cylinder_elem is not None:
                radius = float(cylinder_elem.find('.//radius').text)
                length = float(cylinder_elem.find('.//length').text)
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
                radius = float(sphere_elem.find('.//radius').text)
                obstacles.append({
                    'type': 'sphere',
                    'pose': pose,
                    'radius': radius
                })
    
    return world_size, obstacles

def create_pgm_map(world_size, obstacles, resolution, output_dir, map_name):
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Calculate map dimensions
    width_meters, height_meters = world_size
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
            yaw = pose[5]
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
    
    # Save PGM file
    pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
    img.save(pgm_path, 'PPM')
    
    # Create YAML configuration file
    yaml_path = os.path.join(output_dir, f"{map_name}.yaml")
    with open(yaml_path, 'w') as yaml_file:
        yaml_file.write(f"image: {map_name}.pgm\n")
        yaml_file.write(f"resolution: {resolution}\n")
        yaml_file.write(f"origin: [{-width_meters/2}, {-height_meters/2}, 0.0]\n")
        yaml_file.write("occupied_thresh: 0.65\n")
        yaml_file.write("free_thresh: 0.196\n")
        yaml_file.write("negate: 0\n")
    
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
        world_size, obstacles, args.resolution, args.output_dir, args.map_name
    )
    
    print(f"Map created successfully!")
    print(f"PGM file: {pgm_path}")
    print(f"YAML file: {yaml_path}")

if __name__ == "__main__":
    main()