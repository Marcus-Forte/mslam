# mslam: Minimal Slam by Marcus

## TODO
- Have sensors store timestamp at time of sampling.
- Finish voxel hashing
- Implement point-to-line error model
- Implement acceleration and velocity model


## Registering individual scans 

This executable register two point clouds and publish their results in the GL server (https://github.com/Marcus-Forte/learning-opengl) 
`./register_scans host.docker.internal:50051 target.ply src1.ply src2.ply ...`

Data is available at `test/data/scan*.ply`