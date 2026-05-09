# TODO list

## Slam 

* Documentation
* slam unit test
* Add initial scan logic to slam class?
* SO3 operations module
* Publish map increments to SlamServer service
* Remove slam player delay config from json, make it a program argument
* Fix player so that it returns sensor data accordingly.
* Outlier pre-processing (e.g. radius outlier removal)
* Voxel hashing adjacent voxel configuration
* Logging to stages of the pipeline

## Client

* Have client attempt reconnection if connection to server is lost.
* Scan Recoder: listen to sensor data and write to disk.

## Devenv

* clang-tidy
* Build Pipeline 