from typing import List, Tuple

import cv2
import numpy as np
import regex as re
from scipy.interpolate import splev, splprep
from scipy.spatial import Delaunay


def read_off(path_off: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    read the vertices and faces of a mesh from an off file
    """
    with open(path_off, 'r') as file:
        if 'OFF' != file.readline().strip():
            raise('Not a valid OFF header')
        n_verts, n_faces, _ = tuple([int(s) for s in file.readline().strip().split(' ')])
        vert_count = 0
        verts = []
        faces = []
        while vert_count < n_verts:
            s = file.readline()
            if s.isspace() or s[0] == '#':
                continue
            verts.append([float(s) for s in re.sub(r" +", " ", s).strip().split(' ')])
            vert_count += 1
        face_count = 0
        while face_count < n_faces:
            s = file.readline()
            if s.isspace():
                continue
            if len([int(s) for s in re.sub(r" +", " ", s).strip().split(' ')][1:]) == 3:
                faces.append([int(s) for s in re.sub(r" +", " ", s).strip().split(' ')][1:])
            elif len([int(s) for s in re.sub(r" +", " ", s).strip().split(' ')][1:]) == 6:
                line = [int(s) for s in re.sub(r" +", " ", s).strip().split(' ')]
                faces.append(line[1:1+line[0]])

            faces.append([int(s) for s in re.sub(r" +", " ", s).strip().split(' ')][1:])
            face_count += 1
        return np.array(verts), np.array(faces)
    
def write_off(path_off: str, verts: np.ndarray, faces: np.ndarray) -> None:
    """
    write the vertices and faces of a mesh to an off file
    """
    with open(path_off, 'w+') as file:
        file.write('OFF\n')
        file.write(f'{verts.shape[0]} {len(faces)} 0\n')
        file.write('')
        for v in verts:
            file.write(f'{v[0]} {v[1]} {v[2]}\n')
        for f in faces:
            face = f'{len(f)} {" ".join([str(i) for i in f])}\n'
            file.write(face)
    print(f'Written to {path_off}')

def mesh_to_grid(verts: np.ndarray, res: int=2) -> np.ndarray:
    """
    get verts and faces of a mesh and update the indexing of the vertices to be in a grid
    and also update the faces with the new indexing
    """
    x_size = int(np.sqrt(verts.shape[0]))
    z_size = int(np.sqrt(verts.shape[0]))

    # res sets the space between the vertices in the grid
    # 4: position + original index
    grid = np.zeros(((res+1)*(x_size-1)+1, (res+1)*(z_size-1)+1, 4))

    # store origianl indecies of the vertices as the new column in verts
    verts = np.hstack((verts, np.arange(verts.shape[0]).reshape(-1, 1)))

    # sort the vertices by x and then z and add them to the grid
    verts = verts[verts[:, 0].argsort()]
    for i in range(int(x_size)):
        verts[i * int(z_size):(i + 1) * int(z_size)] = verts[i * int(z_size):(i + 1) * int(z_size)][
            verts[i * int(z_size):(i + 1) * int(z_size)][:, 2].argsort()]
        for j in range(int(z_size)):
            grid[(res+1)*i, (res+1)*j] = verts[i * int(z_size) + j]
    return grid

def get_road_skeleton(road_png: str, grid_shape, threshold: int) -> np.ndarray:
    """
    get the road png and return the road skeleton
    """
    img = cv2.imread(road_png, cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(img, (grid_shape[1], grid_shape[0]))
    img = cv2.bitwise_not(img) # invert image
    
    size = img.shape
    skel = np.zeros(size, np.uint8) # create empty image to store skeleton

    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)) # define the structuring element

    while True:
        open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        temp = cv2.subtract(img, open)
        eroded = cv2.erode(img, element)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        if cv2.countNonZero(img)==0:
            break
    # convert skel to binary image
    skel[skel > threshold] = 255

    return skel


def shift_curve(curve: np.ndarray,  
                control_points_no: int=20, 
                shift_width: int=10, 
                just_border: bool=True) -> np.ndarray:

    # Extract the coordinates of the curve
    curve_coords = np.array([[x, y] for x, y in zip(curve.nonzero()[0], curve.nonzero()[1])])
    
    # Down sample the number of points
    curve_coords = curve_coords[::len(curve_coords)//control_points_no]
    
    # Fit a spline to the curve
    tck, u = splprep(curve_coords.T, k=1, s=0)
    
    # Compute the normal vectors
    dx, dy = splev(u, tck, der=1)
    normal_vectors = np.column_stack((-dy, dx))
    normal_vectors /= np.linalg.norm(normal_vectors, axis=1, keepdims=True)

    shifted_curve_cont = np.zeros_like(curve)
    shifted_curve_disc = np.zeros_like(curve)
    
    res_cont = []
    res_disc = []

    if just_border:
        shift_range = np.array([-shift_width, 0, shift_width])
        # res.append(curve_coords)
    else:
        shift_range = np.arange(-shift_width, shift_width+1)

    for i in shift_range:
        shift_vectors = i * normal_vectors
        shifted_curve_coords = curve_coords + shift_vectors.astype(int)
        # remove out of bounds points
        shifted_curve_coords = \
        shifted_curve_coords[(shifted_curve_coords[:, 0] >= 0) & 
                             (shifted_curve_coords[:, 0] < curve.shape[0]) & 
                             (shifted_curve_coords[:, 1] >= 0) & 
                             (shifted_curve_coords[:, 1] < curve.shape[1])]
        cv2.polylines(shifted_curve_cont, [shifted_curve_coords.astype(int)], isClosed=False, color=255, thickness=1)
        shifted_curve_disc[shifted_curve_coords[:, 0], shifted_curve_coords[:, 1]] = 255
        cont = np.array([[x, y] for x, y in zip(shifted_curve_cont.nonzero()[0], shifted_curve_cont.nonzero()[1])])
        disc = np.array([[x, y] for x, y in zip(shifted_curve_disc.nonzero()[0], shifted_curve_disc.nonzero()[1])])
        res_cont.append(cont)
        res_disc.append(disc)
    
        shifted_curve_cont = np.zeros_like(curve)
        shifted_curve_disc = np.zeros_like(curve)

    return res_disc, res_cont


def get_road_verts(road_coords: np.ndarray, 
                    grid: np.ndarray,
                    res: int, 
                    offset: int) -> np.ndarray:
    x_size = (grid.shape[0]-1)//(res+1)+1
    z_size = (grid.shape[1]-1)//(res+1)+1
    last_idx = x_size*z_size
    idx = 0
    verts = []
    new_positions = []
    for i, j in road_coords:
        idx += 1
        pos = get_approx_terrain_pos(grid, i, j, res)
        pos[1] += offset
        verts.append(pos)

    return np.array(verts)


def triangulate_road(roads: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
    verts = []
    faces = []

    # picke first, middle and the last road
    roads = [roads[0], roads[len(roads)//2], roads[-1]]

    road_length = min([len(road) for road in roads])

    starts = []
    for i, p1 in enumerate(roads[0]):
        for j, p2 in enumerate(roads[1]):
            for k, p3 in enumerate(roads[2]):
                # check if p2 is the average of p1 and p3
                if np.allclose(p2, (p1+p3)/2):
                    starts.append([i, j, k])
                    break
    starts = starts[0]

    verts = np.zeros((len(roads)*road_length, 2), dtype=np.int64)
    for i, road in enumerate(roads):
        road = road[starts[i]:starts[i]+road_length]
        verts[i*road_length:(i+1)*road_length, 0] = road[:, 0]
        verts[i*road_length:(i+1)*road_length, 1] = road[:, 1]

    for i in range(road_length-1):
        for j in range(len(roads)-1):
            faces.append([j*road_length+i, j*road_length+i+1, (j+1)*road_length+i+1])
            faces.append([j*road_length+i, (j+1)*road_length+i+1, (j+1)*road_length+i])
    return np.array(verts), np.array(faces)


def get_neighbours(grid, i, j, res):
    # check for boundary conditions
    if i == grid.shape[0]-1 or j == grid.shape[1]-1:
        i = i - 1
        j = j - 1
    ii = (res+1)*(i//(res+1))
    jj = (res+1)*(j//(res+1))

    neighbours = [[ii, jj], 
                  [ii+(res+1), jj], 
                  [ii, jj+(res+1)], 
                  [ii+(res+1), jj+(res+1)]]
    return np.array(neighbours)


def get_mesh_nearby(grid_size, i, j, res):
    if i == grid_size[0]-1 or j == grid_size[1]-1:
        i = i - 1
        j = j - 1
    ii = (res+1)*(i//(res+1))
    jj = (res+1)*(j//(res+1))
    return ii, jj    


def get_approx_terrain_pos(grid: np.ndarray, i: int, j: int, res: int) -> np.ndarray:
    # check for boundary conditions
    if i == grid.shape[0]-1 or j == grid.shape[1]-1:
        i = i - 1
        j = j - 1
    ii = (res+1)*(i//(res+1))
    jj = (res+1)*(j//(res+1))

    neighbours = [grid[ii, jj], 
                  grid[ii+(res+1), jj], 
                  grid[ii, jj+(res+1)], 
                  grid[ii+(res+1), jj+(res+1)]]
    neighbours = np.array(neighbours)
    neighbours = neighbours[:, :3]
    pos = np.average(neighbours, axis=0) #, weights=get_weights(i, j, ii, jj, res))
    return pos

def get_weights(i: int, j: int, ii: int, jj: int, res: int) -> List[float]:
    weights = [np.sqrt((i-u)**2+(j-v)**2) for u, v in \
               [(ii, jj), (ii+(res+1), jj), (ii, jj+(res+1)), (ii+(res+1), jj+(res+1))]]
    max_dist = np.sqrt(2*(res+1)**2)
    weights = [d/max_dist for d in weights]
    return weights


def patch_road_to_mesh(grid, roads_cont, roads_disc, res, offset):

    # set the vertices behind the road to -1
    # create delaunay triangulation of the terrain
    # remove triangles that are behind the road, update the vertices and faces
    # triangulate the road
    # connect the road to the terrain
    # return the vertices and faces
    
    # mark off the vertices behind the road
    # those inside with -1
    # those on the boundary with -2
    left_road = roads_cont[0]
    right_road = roads_cont[-1]
    curve = np.vstack((left_road, right_road[::-1]))
    curve_coords = np.array([[j, i] for i, j in curve])
    # ray casting algorithm
    for i in range(grid.shape[0]):
        boundaries = curve_coords[curve_coords[:, 0] == i][:, 1]
        boundaries = collapse_sequence(boundaries)
        # print(boundaries)
        for j in range(grid.shape[1]):
            # if len(boundaries) % 2 == 0:
            if len(boundaries) >= 2:
                for k in  range(0, len(boundaries), 2):
                    if boundaries[k][0] <= j <= boundaries[k+1][1]:
                        neighbours = get_neighbours(grid, i, j, res)
                        if j > boundaries[k][0] and j < boundaries[k+1][1] \
                            and [i, j] in get_neighbours(grid, i, j, res).tolist():
                            grid[i, j, 3] = -1
                        for neighbour in neighbours:
                            grid[neighbour[0], neighbour[1], 3] = -1 if grid[neighbour[0], neighbour[1], 3] != -2 else -2
                            # mark the bounday points as -2
                            if (neighbour[0] == i and neighbour[1] == boundaries[k][0]) \
                            or (neighbour[0] == i and neighbour[1] == boundaries[k+1][1]):
                                grid[neighbour[0], neighbour[1], 3] = -2
                        # mark boundary points
                        x, y = None, None
                        if j == boundaries[k][0]: # left boundary
                            x, y = get_mesh_nearby(grid.shape, i, j, res)
                        elif j == boundaries[k+1][1]: # right boundary
                            x, y = get_mesh_nearby(grid.shape, i+res, j+res, res)
                        if x:
                            grid[x, y, 3] = -2
    
    # create delaunay triangulation of the terrain
    # only consider the vertices with positive values
    # and boundary vertices with -2
    nonboundary_verts = grid[grid[:, :, 3] > 0][:, :3]
    boundary_verts = grid[grid[:, :, 3] < 0][:, :3]
    boundary_verts[:, 1] = offset
    terrain_verts = np.vstack((nonboundary_verts, boundary_verts))
    terrain_xz_verts = np.vstack((terrain_verts[:, 0], terrain_verts[:, 2])).T
    terrain_triangulation = Delaunay(terrain_xz_verts)
    terrain_faces = terrain_triangulation.simplices
    return terrain_verts, terrain_faces, boundary_verts
    # remove faces with vertices all from boundary
    terrain_faces = terrain_faces[np.all(terrain_faces < len(nonboundary_verts), axis=1)]
    
    # trinagulate the road
    road_grid, road_faces = triangulate_road(roads_disc)
    road_verts = get_road_verts(road_grid, grid, res, offset)
    terrain_verts = np.vstack((terrain_verts, road_verts))

    # append road faces to terrain faces
    terrain_faces = np.vstack((terrain_faces, road_faces+len(terrain_verts)-len(road_verts)))

    # connect the road to the terrain
    # find two closest boundary vertices to the road
    boundary_idxs = list(range(len(nonboundary_verts), len(terrain_verts)-len(road_verts)))
    road_idxs = list(range(len(terrain_verts)-len(road_verts), len(terrain_verts)))
    for i in range(len(road_idxs)):
        dists = np.linalg.norm(terrain_verts[boundary_idxs] - terrain_verts[road_idxs[i]], axis=1)
        closest = np.argsort(dists)[:2]
        terrain_faces = np.vstack((terrain_faces, [boundary_idxs[closest[0]], boundary_idxs[closest[1]], road_idxs[i]]))

    return terrain_verts, terrain_faces


def collapse_sequence(seq):
    seq = list(seq)
    seq = sorted(seq)
    seq.reverse()
    boundaries = []
    while len(seq) != 0:
        start = seq.pop()    
        element = start
        if len(seq) != 0:
            while seq[-1] == element+1:
                element = seq.pop()
                if len(seq) == 0:
                    break
        end = element
        boundaries.append((start, end))
    return boundaries
