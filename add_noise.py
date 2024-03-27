import click
import numpy as np
import open3d as o3d

@click.command()
@click.option("-m", "--mesh_name", required=True, default="drill.ply")
def main(mesh_name):
    # read mesh file and prepare for visualization
    mesh = o3d.io.read_triangle_mesh(f"data/mesh/{mesh_name}")
    rot = mesh.get_rotation_matrix_from_xyz((0, np.pi, 0))
    mesh.rotate(rot, center=mesh.get_center())
    mesh.compute_vertex_normals()

    # add noise to the mesh
    vertices = np.asarray(mesh.vertices)
    v_normals = np.asarray(mesh.vertex_normals)
    noise = 1e-4 * 2 * (np.random.rand(*vertices.shape) - 0.5) * v_normals
    noised_vertices = vertices + noise
    
    # update the noised mesh
    mesh.vertices = o3d.utility.Vector3dVector(noised_vertices)
    mesh.compute_vertex_normals()

    # save
    o3d.io.write_triangle_mesh(f"data/mesh/noised_{mesh_name}", mesh)


if __name__ == "__main__":
    main()