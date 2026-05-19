# TODO list

## Slam 

* Documentation
* Feature Extraction
  * Outlier pre-processing (e.g. radius outlier removal)
* Correction Heuristics (e.g. loop closure, keyframe selection, fitness score)
* Unit Testing
* Improve Initialization (currently just uses first scan as map)
* Fix player so that it returns sensor data accordingly (time?)
* Keep / Use point intensities for registration? (currently ignored)
* Dense map storing and publishing (option)
* Better grouping of SO3 and SE3 operations. Can be encapsulated in moptim.
* Cleanup preinegration
* Abstraction and Parallelism for per-point processing tasks.

## Devenv

* clang-tidy
* Build Pipeline 