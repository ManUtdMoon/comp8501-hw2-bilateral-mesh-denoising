# COMP8501-HW2: Bilateral Mesh Denoising
Dongjie Yu (3030102216)

## Project Structure
```
├── README.md
├── add_noise.py (add noise to mesh, only drill needs it)
├── bmd.py (main script)
├── data (noisy or denoised mesh)
│   └── mesh
│       ├── denoised_iter_0_noised_bunny.ply
│       ├── denoised_iter_0_noised_drill.ply
│       ├── denoised_iter_1_noised_bunny.ply
│       ├── denoised_iter_1_noised_drill.ply
│       ├── denoised_iter_2_noised_bunny.ply
│       ├── denoised_iter_2_noised_drill.ply
│       ├── drill.ply
│       ├── noised_bunny.ply
│       └── noised_drill.ply
├── download.py (deprecated, download reconstructed mesh from redwood)
├── redwood_3dscan (deprecated)
├── requirements.txt
└── visualize_denoising.py (visualize mesh, not only denoised ones)
```

## Mesh File Credit
- `drill.ply` comes from [The Stanford 3D Scanning Repository](https://graphics.stanford.edu/data/3Dscanrep/).
  - Then I added noise to it using `python add_noise.py` to generate `noised_drill.ply`.
- `noised_bunny.ply` comes from [this github repo](https://github.com/daweidavidwang/Mesh_Denoiseing_BilateralFilter).

## Installing environment
Install the required packages by running the following command:
```bash
pip install -r requirements.txt
```

## Run
### Preview Mesh
Run the following command to visualize the noised mesh:
```bash
python visualize_denoising.py -m MESH_NAME
```
where `MESH_NAME` is either `noised_bunny.ply` or `noised_drill.ply`.

### Denoising
Run the following command to denoise the mesh:
```bash
python bmd.py -m MESH_NAME
```
where `MESH_NAME` is either `noised_bunny.ply` or `noised_drill.ply`. By default, the script will run 3 iterations of denoising. The denoised mesh will be saved in the `data/mesh` folder in the name of `denoised_iter_{i}_{MESH_NAME}`.

It takes about $9=3\times3$ seconds to denoise `noised_drill.ply` and $60=3\times20$ minutes to denoise `noised_bunny.ply` due to the large number of vertices.

### Visualize Denoising
Run the following command to visualize the denoised mesh:
```bash
python visualize_denoising.py -m MESH_NAME
```
where `MESH_NAME` is either `denoised_iter_0_noised_bunny.ply`, `denoised_iter_1_noised_bunny.ply`, `denoised_iter_2_noised_bunny.ply`, `denoised_iter_0_noised_drill.ply`, `denoised_iter_1_noised_drill.ply`, or `denoised_iter_2_noised_drill.ply`.

## Acknowledgements
Following repos are referred to:
- [daweidavidwang-bmd](https://github.com/daweidavidwang/Mesh_Denoiseing_BilateralFilter)
