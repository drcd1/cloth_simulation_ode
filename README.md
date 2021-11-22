# Cloth simulation
## A Project for the Numerical Algorithms for ODE's at the University of Saarland

![A cloth falls on an invisible sphere](https://github.com/drcd1/cloth_simulation_ode/blob/main/media/output.gif)

## Instructions
(The following instructions are intended for a Linux computer)

### How to compile:
`g++ -o prog clothSimulation/src/main.cpp -O3`
OR
`make`

### How to run
`./prog`

The program will then ask you to input the parameters you'd like to use.
(cloth resolution, integrator, timestep, etc...)


The rendering resolution can be setup by changing the the value RES defined in clothSimulation/src/main.cpp and recompiling.
Rendered images will be saved in `.ppm`(Portable Pixel Map).

From an image sequence, one can use external software to generate a video. We sugest using ffmpeg:
If the image format has the form `<fileprefix>%<n>d.ppm` (example: `fileprefix` = "image",`n=3` - image000.ppm, image001.ppm,etc),
the following command may be used.
`ffmpeg -i image%3d.ppm -b:v 8M -r 30 video.mp4`


Note: if the rendering is taking too long for later frames, use Ctrl-C to quit the program.
 If you inspect the results, it's likely that the simulation "exploded".
 If the simulation "explodes", it may cause the program to crash with a "segmentation fault".

Note 2: the files args.txt can be used as input to the program to simulate and render one of the default scenes:
`./prog<args.txt`

## Results

You may find the results of this short experiment in this [report](https://github.com/drcd1/cloth_simulation_ode/blob/main/media/report.pdf) and this [video](https://www.youtube.com/watch?v=FVa_d85nywg).

