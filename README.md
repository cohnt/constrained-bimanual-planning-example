# Constrained Bimanual Planning Example Repository

This repository provides example code for **planning motions in a bimanual manipulation setup**, where the **relative transform between the two end-effectors must remain fixed**.
The approach and concepts are based on our [ICRA 2024 paper](https://ieeexplore.ieee.org/abstract/document/10610675/).

Our formulation uses **analytic inverse kinematics (IK)** to parameterize the constraint manifold, producing a **minimal coordinate system**.
In this intrinsic parameterization, the kinematic equality constraint is eliminated entirely.
By applying **automatic differentiation** through this mapping (via the chain rule), we can propagate gradients seamlessly and solve optimization problems involving costs, constraints, and variables expressed in **both the parameterized space and the full configuration space**.
This leads to simpler optimization problems whose resulting trajectories satisfy the bimanual kinematic constraint **by construction**.

Since the publication of the ICRA paper, we have substantially improved the supporting infrastructure through numerous contributions to [Drake](https://drake.mit.edu/).
In parallel, we developed new algorithms for constructing **collision-free polytopes** described in [a publication](https://arxiv.org/abs/2410.12649) presented at ISRR 2024.
This repository serves primarily as a **tutorial and demonstration** of how to use these new tools together.

---

## Features Demonstrated

- **Parameterization Construction**
  - Creation of an analytic mapping as a Python function compatible with [Drake’s automatic differentiation](https://github.com/RobotLocomotion/drake/blob/v1.46.0/tutorials/autodiff_basics.ipynb).
  - We use a classic leader-follower setup for our parameterization, including the self-motion of the follower arm. The degrees-of-freedom are visualized in [this picture](./other/degrees_of_freedom.jpg).
- **Constraint Formulation**
  - Construction of **reachability** and **subordinate arm joint limit** constraints in the parameterized space.
- **Convex Region Generation**
  - Use of [`IrisNp2`](https://drake.mit.edu/doxygen_cxx/group__planning__iris.html#gaf5bc571d0ee3753c976d3b521de397c4) and [`IrisZo`](https://drake.mit.edu/doxygen_cxx/group__planning__iris.html#ga9b44245010bfdc8163645f0c62f9e9ab) to build convex regions in the parameterized space that are **collision-free** and **kinematically valid**.
  - These algorithms will, with a user-specified probability, produce a polytope such that a target percentage of the region is collision-free and kinematically valid.
- **Motion Planning**
  - **Trajectory Optimization using Graph of Convex Sets:** using [`GcsTrajectoryOptimization`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_gcs_trajectory_optimization.html).
  - **Sampling-Based Planning:** bidirectional RRT followed by shortcutting in the parameterized space.
  - **Kinematic Trajectory Optimization:** refining RRT results using [`KinematicTrajectoryOptimization`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_kinematic_trajectory_optimization.html).
  - **Representing Trajectories:** transforming trajectories in the parameterized space back to the full configuration space using [`FunctionHandleTrajectory`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_function_handle_trajectory.html).
  - **Dynamic Retiming:** retiming trajectories with [`Toppra`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_toppra.html) to enforce velocity and acceleration limits.

---

## Getting Started

### Installation

Probably the only dependency you'll need to install is [**Drake**](https://drake.mit.edu/).
You can install the latest stable release directly from PyPI with `pip install drake`.

If you want to use the [C++ implementation](./cpp_parameterization) of the parameterization, you'll have to [build Drake from source](https://drake.mit.edu/from_source.html).
The C++ implementation gives major speedups, bringing region generation down to ~1.2 seconds on my laptop!
Similarly, trajectory optimization only takes a few seconds, even with the computationally heavy parameterized costs.

At the time of this repository's release, some of the features are only recently merged into Drake, so you may need to install a nightly build.
Check out [Drake's installation instructions](https://drake.mit.edu/pip.html) for more details.
This should no longer be an issue once Drake v1.48.0 has released. ([Expected around mid-December 2025](https://drake.mit.edu/release_notes/release_notes.html).)

## Contents

- **models/** -- Contains all robot, environment, and object models used in the examples.
  - `models/iiwa14_convex_decimated_collision.urdf` -- URDF model of the KUKA IIWA-14 arm. Collision geometries have been convexified, and then simplified with quadratic edge collapse decimiation in Meshlab.
  - `models/old_shelves.dmd.yaml` -- **Scene description** defining the default environment layout for the examples.
- **notebooks/** -- Contains Jupyter notebooks demonstrating example workflows.
  - `notebooks/main.ipynb` -- Main tutorial notebook demonstrating the bimanual planning workflow end-to-end.
- **src/** -- Python source code implementing the core functionality of the examples.
  - `src/iiwa_analytic_ik.py` -- Analytic inverse kinematics implementation for the IIWA arm, used to construct the intrinsic parameterization.
  - `src/rrt.py` -- Simple bidirectional RRT implementation for sampling-based planning in the parameterized space.
  - `src/shortcut.py` -- Simple randomized shortcutting implementation for improving a piecewise-linear path.

### Implementation Notes

- Many components can be ported to C++ for significant speedups (most notably, the callable function used by IrisParameterizationFunction), but everything can be run using just Python for accessibility and clarity.
- You can find a C++ implementation of the parameterization, costs, and constraints in [`./cpp_parameterization`](./cpp_parameterization). This requires a source build of Drake to compile.
- Because Python code cannot easily be called in parallel from C++, IRIS-ZO is not highly performant without the C++ parameterization.
- The sampling-based planning baselines (e.g., RRT) are simple and illustrative, intended for comparison rather than performance.

I've also done my best to include comments in the tutorial notebook and python files to add further clarity.

## References

If you use the motion planning and minimal coordinates aspects, please cite:
```
@inproceedings{cohn2024constrained,
  title={Constrained bimanual planning with analytic inverse kinematics},
  author={Cohn, Thomas and Shaw, Seiji and Simchowitz, Max and Tedrake, Russ},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={6935--6942},
  year={2024},
  organization={IEEE},
}
```
If you use IRIS-NP2 or IRIS-ZO, please cite:
```
@article{werner2024faster,
  title={Faster Algorithms for Growing Collision-Free Convex Polytopes in Robot Configuration Space},
  author={Peter Werner and Thomas Cohn* and Rebecca H. Jiang* and Tim Seyde and Max Simchowitz and Russ Tedrake and Daniela Rus},
  year={2024},
  journal={arXiv preprint arXiv:2410.12649},
  addendum={*Denotes equal contribution.}
}
```
If you specifically use IRIS-NP2 or IRIS-ZO in minimal coordinates, please cite our workshop paper:
```
@inproceedings{cohn2025,
  title={Faster Algorithms for Growing Collision-Free Regions of Constrained Bimanual Configuration Spaces},
  author={Thomas Cohn and Peter Werner and Russ Tedrake},
  year={2025},
  maintitle={Frontiers in Dynamic, Intelligent, and Adaptive Multi-Arm Manipulation},
  booktitle={2025 International Conference on Intelligent Robots and Systems (IROS)},
}
```
(A journal version, which encompasses the region generation in minimal coordinates, is in preparation.)

If you use GcsTrajectoryOptimization, please cite:
```
@article{marcucci2023motion,
  title={Motion planning around obstacles with convex optimization},
  author={Marcucci, Tobia and Petersen, Mark and von Wrangel, David and Tedrake, Russ},
  journal={Science Robotics},
  volume={8},
  number={84},
  pages={eadf7843},
  year={2023},
  publisher={American Association for the Advancement of Science}
}
```

## Contact
For questions or suggestions, please open an issue or contact [Thomas Cohn](mailto:tcohn@mit.edu).