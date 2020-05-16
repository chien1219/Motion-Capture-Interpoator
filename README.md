# Motion-Capture-Interpoator
CSCI-520 2020 Spring Project2

+ Implement Bezier interpolation for Euler angles.
+ Represent the joint angles (including root orientation) using quaternions. A quaternion library is included with the starter code. This + library can perform quaternion arithmetics, and also convert from quaternions to rotation matrices and back. You must write a routine that converts from Euler angles to rotations. Starter code already provides a routine that converts from rotations to Euler angles; in XYZ angle format. Then, use these routines to write routines that convert from Euler angles to quaternions, and back (see interpolator.cpp).
+ Implement Spherical Linear (SLERP) interpolation for quaternions. For implementation details, see the course slides, Rick Parent's "Computer Animation" book (pages 97-102 (first edition), or pages 110-114 (second edition)), and Ken Shoemake's "Animating Rotation with Quaternion Curves" paper.
+ Implement Bezier Spherical Linear (SLERP) interpolation for quaternions (see same references as above for help). Primarily, we recommend that you follow the material in Rick Parent's book.
+ The Bezier control points a_n and b_n+1 must be pushed towards points p_n and p_n+1 (1/3 factor; see the notes and Shoemake paper). This applies both to Euler Bezier and Quaternion Bezier.
+ With Quaternion interpolation, use Euler interpolation to interpolate the root position, as follows. Linear Quaternion should use Linear Euler for the root, and Bezier Quaternion should use Bezier Euler for the root.
