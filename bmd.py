import open3d as o3d
import numpy as np
import click


def get_vertex_neighbours(vertex, vertices, radius):
    """
    Get the vertices within a certain radius of a given vertex.

    Parameters
    @vertex: np.ndarray
        The vertex of interest.
    @vertices: np.ndarray
        The vertices array.
    @radius: float
        The radius within which to search for vertices.

    Returns
    @neighbours: list
        The vertices within the radius.
    """
    pass


def denoise_point(vextex, normal):
    """
    Denoise a single point on the mesh.

    Parameters
    @vextex: np.ndarray
        The vertex to denoise.
    @normal: np.ndarray
        The normal of the vertex.

    Returns
    @de_vertex: np.ndarray
        The denoised vertex.
    """
    pass


def bilateral_mesh_denoising(vertices, triangles, v_normals, sigma_s=1, sigma_r=0.1):
    """
    Perform bilateral mesh denoising.

    Parameters
    @vertices: np.ndarray
        Vertices of the mesh.
    @triangles: np.ndarray
        Triangles of the mesh.
    @v_normals: np.ndarray
        Vertex normals of the mesh.
    @sigma_c: float, optional
        Spatial sigma, by default 1.
    @sigma_s: float, optional
        Range sigma, by default 0.1.

    Returns
    @de_vertices: np.ndarray
        Denoised vertices array.
    """
    pass


@click.command()
@click.argument("mesh_id", required=True, default="06457")
def main(mesh_id):
    # read mesh file and prepare for visualization
    mesh = o3d.io.read_triangle_mesh(f"data/mesh/{mesh_id}.ply")
    mesh.compute_vertex_normals()
    rot = mesh.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    mesh.rotate(rot, center=mesh.get_center())

    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    v_normals = np.asarray(mesh.vertex_normals)

    print(
        f"Mesh has \n"
        f"    {len(vertices)} vertices and \n"
        f"    {len(triangles)} triangles."
    )

    # perform bilateral mesh denoising
    


if __name__ == "__main__":
    main()