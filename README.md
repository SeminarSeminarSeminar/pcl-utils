# pcl-utils

Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html

## Prerequisites

Point Cloud Library


## Build
```
$ mkdir build && cd build
$ cmake ..
```

Sample .pcd files are located in the `samples` directory.


## Concatenate multiple .pcd files

### Usage
```
$ ./concat [output file] [input file 1] [input file 2] ...
```

### Example

## [Filter] Statistical Outlier Removal

### Usage
```
$ ./filter_stat input.pcd output.pcd [meank(default=50)] [stddevmulthreash(default=1.0)]
```
|parameter|default|description|
|---|---|---|
|meanK|50|Set the number of nearest neighbors to use for mean distance estimation.|
|StddevMulThresh|1.0|Set the standard deviation multiplier for the distance threshold calculation.|

### Example

## [Filter] Radius Outlier Removal

### Usage
```
$ ./filter_radius input.pcd output.pcd [RadiusSearch] [MinNeighborsInRadius]
```

|parameter|default|description|
|---|---|---|
|RadiusSearch|0.01|Set the radius of the sphere that will determine which points are neighbors.|
|MinNeighborsInRadius|10|Set the number of neighbors that need to be present in order to be classified as an inlier.|

### Example

## [Filter] Voxel Grid

### Usage
```
$ ./filter_voxel input.pcd output.pcd output_grid.pcd [LeafSize]
```

|parameter|default|description|
|---|---|---|
|LeafSize|0.05|Set the voxel grid leaf size.|

### Example

## [Segmentation] Region Growing

### Usage
```
$ ./seg_region input.pcd output.pcd [NumberOfNeighbors]
```

|parameter|default|description|
|---|---|---|
|NumberOfNeighbors|300|Allows to set the number of neighbours.|

* Each vertical/horizontal surface will be saved to its own .pcd file
(region_vertical_0.pcd, ..., region_horizontal_0.pcd, ...)

* `output.pcd` includes gropus of red points which indicate points that could not be segmented.

### Example

## [Segmentation] Adjacency Graph

### Usage
```
$ ./gen_graph input.pcd 
```

### Example
