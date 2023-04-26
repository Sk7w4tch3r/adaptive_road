import polyscope as ps
from utility import read_off
import  glob
import argparse


def vis(directory):
    ps.init()
    
    for file in glob.glob(directory):
        verts, faces = read_off(file)
        ps.register_surface_mesh(file, verts, faces)

    ps.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--vis_target', type=str, default='output', help='path to the output file')
    args = parser.parse_args()
    vis(args.vis_target)