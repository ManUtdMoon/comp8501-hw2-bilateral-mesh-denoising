import open3d as o3d
import numpy as np
import click


@click.command()
@click.option("-m", "--mesh_name", required=True, default="noised_drill.ply")
def main(mesh_name):
    # read mesh
    mesh = o3d.io.read_triangle_mesh(f"data/mesh/{mesh_name}")
    rot = mesh.get_rotation_matrix_from_xyz((0, np.pi, 0))
    mesh.rotate(rot, center=mesh.get_center())

    # visualize
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])


if __name__ == "__main__":
    main()