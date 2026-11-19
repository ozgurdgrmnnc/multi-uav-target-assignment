# core/ (private)

This folder holds the ROS-independent algorithm modules: target motion model, UKF target
estimation, 3D A* + SciPy SLSQP path smoothing, intercept/loiter geometry and candidate
generation for the assignment.

They are based on my advisor's unpublished research code, so they are **not included** in
this public repository. The ROS 2 nodes import them as `iha_wta.core.*`.
