4DOF Child on a Swing – Symbolic Model
Supplementary MATLAB Code for IEEE CDC 2025 Paper

Paper title:
A Hybrid Orbital Stabilizer With Guaranteed Basin of Attraction for Mechanical Systems With Underactuation Degree One

Authors:
Luiz Dias Navarro and Manfredi Maggiore

Conference:
IEEE Conference on Decision and Control (CDC), 2025

Description:
This Matlab script computes the symbolic dynamics of a 4-degree-of-freedom planar mechanical system modeling a child on a swing. It defines symbolic variables and derives expressions for the mass matrix D(q), Coriolis matrix C(q,qdot), gradient of the potential energy ∇P(q), and input matrix B. These expressions are stored as numerical function handles in a structure called sys.

Model structure:
D(q) * q̈ + C(q, q̇) * q̇ + ∇P(q) = B * u

The script also provides Matlab functions:
- sys.ddq(q, dq, u): second-order dynamics
- sys.xdot(x, u): first-order dynamics
- sys.rc1(q), sys.rc2(q), sys.rc3(q), sys.rc4(q): link center-of-mass positions

How to run:
1. Open MATLAB.
2. Navigate to the folder containing the script.
3. Run:

   child_swing_model

Requirements:
- MATLAB R202x or newer
- Symbolic Math Toolbox

Files:
- child_swing_model.m (main script)
- README.md (this file)

Output:
The script prints the symbolic matrices D, C, ∇P, and B in the console. The structure sys contains the dynamics and kinematics functions needed for simulation or analysis.

License:
Copyright 2025, Luiz Dias Navarro and Manfredi Maggiore.
For academic use only. Please cite the IEEE CDC 2025 paper if you use this code.

Citation:
@inproceedings{NavMag2025,
  author    = {Luiz Dias Navarro and Manfredi Maggiore},
  title     = {A Hybrid Orbital Stabilizer With Guaranteed Basin of Attraction for Mechanical Systems With Underactuation Degree One},
  booktitle = {Proceedings of the IEEE Conference on Decision and Control (CDC)},
  year      = {2025}
}

Contact:
Luiz Dias Navarro: luiz.dias.navarro@mail.utoronto.ca
Manfredi Maggiore: maggiore@ece.utoronto.ca
