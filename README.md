# Axes Identification
This program implements a simple algorithm for the identification of serial-link manipulators axes.

# Dependencies
- Eigen3

# Usage
The program reads data from an external file written in plain text. The file should be formatted as follows:
```
joint_1  joint_2  ...  joint_N  roll  pitch  yaw
------------------------------------------------
0.0      0.1      ...  0.0      0.0   0.0    0.0
0.0      0.1      ...  0.5      0.0   0.0    0.3
0.0      0.1      ...  0.8      0.0   0.0    0.4
0.0      0.4      ...  0.8      0.0   0.2    0.3
```

- The robot may possess an arbitrary amount of joints.
- The header line is optional.
- Only one joint may vary from one row (k) to the other (k+1). Otherwise, the k-th row is deleted.
-- Actually, small movements are allowed as long as they do not surpass a given tolerance.
- The last three columns might be replaced by nine columns containing the elements of the rotation matrix
  in row-major order.

# Installation

From the source directory, run
```sh
$ mkdir build
$ cd build
$ cmake ..
$ make
```
The executable `robot-identification` is placed under `<src_dir>/build`.
