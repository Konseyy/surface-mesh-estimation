# Mesh illumination from equirectunglar depth maps

This is a CLI tool for shading 3D scene meshes generated from point clouds given in equirectangular depth map format.

# Example

```
$ cargo run -r -- -i ./example/inputs/in1.png -a marching_cubes
```

This will open a window with the generated mesh and lighting <br> There are several keyboard shortcuts you can use while the program window is focused:

- `Esc` to exit the program
- `N` to toggle showing the normals of the mesh
- `Space` to toggle automatic camera rotation
- `E` to move the camera up
- `Q` to move the camera down
- `W` to move the camera forwards
- `S` to move the camera backwards
- `A` to rotate the camera left
- `D` to rotate the camera right
- `T` to reset the camera to the starting position
- `Y` to reset the camera the positio of the light source
- `F` to toggle showing shadows

### Disclaimer

The above command runs the tool in release mode, using all available threads, as the calculation is expensive

# Usage

The usage instructions can be seen by running the following command:

```
$ cargo run -- --help
```

or

```
$ cargo run -- -h
```
