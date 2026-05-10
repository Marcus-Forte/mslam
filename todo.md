# TODO list

## Slam 

* Documentation
* slam unit test
* Add initial scan logic to slam class?
* SO3 operations module
* Publish map increments to SlamServer service
* Fix player so that it returns sensor data accordingly.
* Outlier pre-processing (e.g. radius outlier removal)
* Voxel hashing adjacent voxel configuration
* Logging to stages of the pipeline
* Combine sensor timestamps ( deskew)
* Add other services: reset scan, start, stop, etc.

## Client

* Have client attempt reconnection if connection to server is lost.

## Devenv

* clang-tidy
* Build Pipeline 