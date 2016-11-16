# gptam
George's PTAM with OpenCV 

This real-time Visual SLAM application is a deep code modification of the brilliant work by Klein and Murray , "Parallel Tracking and Mapping" (PTAM).

(Very) Brief Overview of the Modifications
-----------------------------------------------------------
Although this certainly looks like PTAM, it is however a major line-to-line hack and despite many similarities (mainly in the interface), it is not the original code in many ways:

1. Many changes were made to the code that implements casual (but certainly non-trivial) SLAM-stuff such as Gauss-Newton optimization, point triangulation, SLAM initialization, etc. For instance, the original PTAM initializes by detecting and decomposing a homography a la Faugeras. The original solver finds 8 homographies, although there are 4 at most; furthermore, PTAM carries over a mistake from Faugeras' paper which mistakes arbitrary scale (by-product of SVD) with the actual distance of the plane from the origin along its normal (which cannot be recovered). The new initialization routine is still homography based, but the algorithm now returns the four solutions (including the case of only two solutions which basically correspond to motion along the normal of the plane). In short, the solution is loosely based on Zhang's observation that the middle singular vector of the homography is orthogonal to both the plane normal and the translation. Note that the new decomposition method eliminated the "p" postfix rotation and translation as well as the dubious scale member "d" from the "HomographyDecomposition" structure as they are no longer required fro the reasons explained above.

Other minor-but not trivial changes include the substitution of large LS matrices with much smaller gram-matrix accumulators, the use of a standard LS method to reconstruct a point (instead of the DLT-like method by Hartley and Zisserman) - although the new code is somewhat longer than the original, it however is an analytical solution to begin with and it spares the SVD as well as it possibly earns a few more decimal places of accuracy.

Another (possibly inconsequential?) mistake is the error computation in the G-N refinement of a homography. The error is falsely (I just can't think of a valid reason except for the case that it was thought of as yet another approximation on a method that uses an approximation by definition) estimated based on by the mapmaker based on the cached results of the measured second-image projection and NOT on the reprojection during the 5 G-N iterations. now the error is recomputed upon each new reprojection and the respective Jacobian field in the homography match structure is rendered useless.

A couple more alterations are, a) the modification of the warping matrix in the Patchfinder (now there are two dividers - instead of the point-depth originally used: for column #1, its the depth of *point-plus-right pixel displacement* and for column #2 is the depth of *point-plus-down pixel displacement*) and, b) The use of analytical camera parameter derivatives in the calibrator (can't imagine why they were not used in the first place; possibly because the intent was to have arbitrary number of parameters?)

2. Other changes concern the software engineering side of things: For starters, NO TooN, NO libCVD and NO GVars and Enter openCV:

Although TooN, libCVD and GVars are brilliant to say the least, they are not well documented online (except for the API reference of course) and examples simply do not exist. It has been much easier to go through the entire code line-by-line, rather than trying to write a working example just from the API reference; interestingly, even when certain functions had clear use in the original PTAM, I would still have to read the code in TooN or libCVD; such an example is "halfSample" which effectively is averaging decimation (I originally used pyramidal downsampling and (thereafter) simple resizing with interpolation as provided by OpenCV and discovered that Rosten's simple routine works better results for some reason and its just plain faster!) .

Thus, I ended-up lifting and modifying code such as the SE3, SO3, SO2, SE2 (TooN), GUI (GVars), FAST (libCVD) classes and the OpenGL interface to provide missing functionality using OpenCV Matx and Mat_ objects.
No need to mention some TooN code was a great mind opener, such as, for instance, the near-zero approximations to the Lie exponential, or the brilliant operator overloading schemes (I created mine, but the essential ideas came straight out of the original code).

Unlike other PTAM spawns (e.g., ETH PTAM or the recent ORBSLAM) I kept the OpenGL interface simply because it's awesome! It took a bit of effort to convert it to work with OpenCV matrices, let alone isolate the useful stuff from the rest of libCVD, but it was worth it! The same goes with GVars.
