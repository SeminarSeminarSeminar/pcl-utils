# pcl-utils

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

