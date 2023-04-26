import argparse

import numpy as np
from utility import (get_road_skeleton, mesh_to_grid, patch_road_to_mesh,
                     read_off, shift_curve, write_off)


def main(args):

    terrain_verts, terrain_faces = read_off("data\\input_terrain\\" + args.terrain_mesh + ".off")

    # aveerage height of the terrain + offset
    height = np.mean(terrain_verts[:,1]) * (1+args.offset)

    terrain_grid = mesh_to_grid(terrain_verts, args.resolution)
    skeleton = get_road_skeleton("data\\sketch\\" + args.sketch_name + ".png", terrain_grid.shape, threshold=255)
    roads_disc, roads_cont = shift_curve(skeleton, args.control_points_no, args.road_width, args.just_border)

    terrain_verts, terrain_faces, boundary_verts = patch_road_to_mesh(terrain_grid, roads_cont, roads_disc, args.resolution, height)

    np.savetxt(f'output\\boundary_verts_{args.sketch_name}.txt', boundary_verts)

    write_off(f'output\\terrain_{args.sketch_name}.off', terrain_verts, terrain_faces)
    print(f'done with {args.sketch_name}')
    
    return 0


if __name__ == '__main__':
    # read arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--terrain_mesh', type=str, help='path to the terrain mesh')
    parser.add_argument('--sketch_name', type=str, help='path to the road sketch')
    parser.add_argument('--resolution', type=int, default=5, help='resolution of the terrain')
    parser.add_argument('--road_width', type=int, default=5, help='width of the road')
    parser.add_argument('--offset', type=float, default=0.01, help='offset of the road')
    parser.add_argument('--control_points_no', type=int, default=10, help='number of control points')
    parser.add_argument('--just_border', type=bool, default=True, help='just the border of the road')
    args = parser.parse_args()
    main(args)