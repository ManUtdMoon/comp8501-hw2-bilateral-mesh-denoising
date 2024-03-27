import open3d as o3d
import numpy as np
import click
import tqdm


def bilateral_mesh_denoising(mesh):
    """
    Perform bilateral mesh denoising.

    Parameters
    @mesh
        vertices: np.ndarray (float)
            Vertices of the mesh (location).
        v_normals: np.ndarray (float)
            Vertex normals of the mesh.

    Returns
    @de_vertices: np.ndarray
        Denoised vertices array.
    """
    # prepare
    mesh.compute_vertex_normals()
    mesh.compute_adjacency_list()
    vertices = np.asarray(mesh.vertices)
    v_normals = np.asarray(mesh.vertex_normals)
    assert len(vertices) == len(v_normals)
    kdtree = o3d.geometry.KDTreeFlann(mesh)

    de_vertices = np.zeros_like(vertices)
    pbar = tqdm.tqdm(
        vertices, leave=False, mininterval=1
    )
    for i, vertex in enumerate(pbar):
        # skip isolated vertices
        this_neighbour_indices = list(mesh.adjacency_list[i])
        if len(this_neighbour_indices) == 0:
            de_vertices[i] = vertex
            continue

        # compute sigma_c: the minimum distance to the neighbours
        this_neighbour_vertices = vertices[this_neighbour_indices]
        dists = np.linalg.norm(this_neighbour_vertices - vertex, axis=1)
        sigma_c = np.min(dists)
        radius = 2 * sigma_c
        assert sigma_c.shape == ()

        # compute sigma_s
        ## find neighbours within the radius
        _, neightbours_idx, _ = kdtree.search_radius_vector_3d(vertex, radius)

        neighbours = vertices[neightbours_idx[1:]] # exclude itself
        offsets = np.abs(np.dot(vertex - neighbours, v_normals[i]))
        sigma_s = np.std(offsets)
        sigma_s = np.maximum(sigma_s, 1e-6)
        assert sigma_s.shape == ()

        # weighted sum
        diffs = neighbours - vertex # (B, 3), vertex should move towards neighbours so vector starts from vertex
        ds = np.linalg.norm(diffs, axis=1)  # (B,)
        dr = np.dot(diffs, v_normals[i]) # (B,)

        w = np.exp(-ds**2 / 2 / sigma_c**2) * np.exp(-dr**2 / 2 / sigma_s**2) # (B,)
        summation = np.sum(w * dr)
        normalizer = np.sum(w) + 1e-6 # avoid division by zero
        de_vertices[i] = vertex + v_normals[i] * summation / normalizer

    # update mesh
    mesh.vertices = o3d.utility.Vector3dVector(de_vertices)
    mesh.compute_vertex_normals()
    
    return mesh


def bmd_n_iter(mesh, n_iter, mesh_name):
    """
    Perform bilateral mesh denoising for n iterations.
    """
    for i in range(n_iter):
        mesh = bilateral_mesh_denoising(
            mesh
        )
        o3d.io.write_triangle_mesh(f"data/mesh/denoised_iter_{i}_{mesh_name}.ply", mesh)
    return mesh


@click.command()
@click.option("-m", "--mesh_name", required=True, default="noised_drill.ply")
def main(mesh_name):
    # read mesh file and prepare for visualization
    mesh = o3d.io.read_triangle_mesh(f"data/mesh/{mesh_name}")
    mesh.compute_vertex_normals()

    # some stats
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    print(
        f"Mesh has \n"
        f"    {len(vertices)} vertices and \n"
        f"    {len(triangles)} triangles."
    )

    # perform bilateral mesh denoising
    mesh_name = mesh_name.split(".")[0]
    new_mesh = bmd_n_iter(mesh, 3, mesh_name)
    new_mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([new_mesh])


if __name__ == "__main__":
    main()