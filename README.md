# PTAM with OpenCV (gptam)
 

This real-time Visual SLAM application is a deep code modification of the brilliant original work by Klein and Murray , "Parallel Tracking and Mapping" (PTAM).

Brief Overview of the Additions / Modifications
-----------------------------------------------
Although this certainly looks like PTAM, **it is however a major line-to-line hack and despite many similarities (mainly in the interface), it is not the original code in many ways**. In fact, what is mostly inherited from the original is the OpenGL based interface. Vision algorithms have been re-written either on the basis of the original or completely from scratch:

1. __Algorithmic Alterations__: Many changes were made to the code that implements casual (but certainly non-trivial) SLAM-stuff such as Gauss-Newton optimization, point triangulation, SLAM initialization, etc. 

  * For instance, the original PTAM initializes by **detecting and decomposing a homography a la Faugeras which recovers 8 homographies, although there are 4 at most**; furthermore, this implementation carries over a mistake from Faugeras' paper which **confuses arbitrary scale (by-product of SVD) with the actual distance of the plane from the origin along its normal (which cannot be recovered)**. I implemented a new routine which is still homography based, but the algorithm now returns the four solutions (including the case of only two solutions which basically corresponds to motion along the normal of the plane). In short, the solution is loosely based on Zhang's observation that the middle singular vector of the homography is orthogonal to both the plane normal and the translation. Note that the new decomposition method eliminated need to have the "p" postfixed rotation and translation fields as well as the dubious scale field "d" from the "HomographyDecomposition" structure as they are no longer required for the reasons explained above.

* Other minor-but not trivial changes include the **substitution of large LS matrices with much smaller gram-matrix accumulators**, the use of a **standard LS method to reconstruct a point (instead of the DLT-like method by Hartley and Zisserman)** - although the new code is somewhat longer than the original, it however is an analytical solution to begin with and it spares the SVD as well as it possibly earns a few more decimal places of accuracy.

* Another (possibly inconsequential?) mistake is **the error computation in the G-N refinement of a homography**. The error is falsely (I just can't think of a valid reason except for the case that it was thought of as yet another approximation on a method that uses an approximation by definition) estimated by the mapmaker based on the cached results of the measured second-image projection and NOT on the reprojection during the 5 G-N iterations. **In my version, the error is recomputed upon each new reprojection** and the respective Jacobian field in the homography match structure is rendered useless. I have also added two fields (only one of which is necessary) that hold the actual image positions in Source and Target keyframes. 

A few more alterations that I can think of are, 

* The modification of the warping matrix in the `Patchfinder` (now there are two dividers - instead of the point-depth originally used: for column 1, its the depth of __point-plus-right pixel displacement__ and for column 2 is the depth of __point-plus-down pixel displacement__ ( more explanations can be found in the comments inside `CalcLevelAndWarpMatrix` of the `Patchfinder.cpp` file.)

* The **use of analytical camera parameter derivatives** in the calibrator (can't imagine why they were not used in the first place; possibly because the intent was to have arbitrary number of parameters?). Anyways, I observed that they typically are faster (i.e., the analytical ones) at first, but they slow down in exactly the same way as the numerical derivatives do when the squared error is below 1, so this not much of a win...

* Yet another small alteration concerns the computation of the "central" point from which the **calibrator** detects the **first grid candidate corner** (closest free-lying FAST feature) which is estimated as the barycenter of the free-corners instaed of the center of the image.

* Also, I have used **rotating mirrored cone-like regions** instead of a single scan-line in the original inside the `GuessInitialPose` function to make it more robust. I think in conjunction with the previous calibrator improvement, albeit intuitive changes, they made the grid detection process more robust. 

* I added a **negative depth test for added mappoints**.

* The **bundle adjuster** is effectively the same, although a lot of changes in the code were made. I added a more elaborate condition to che check **whether the `V*` matrix is Positive Semidefinite (PSD)** using a simple implementation of the Cholesky demposition for 3x3 PSD matrices. Also, there is an extra check for **near-zero deterninant** in order to potentially capture vanishing gradient effects on the subsequent positive smidefitiveness of the the matrix `S`. All in all, this condition seems to yield more stable camera pose estimates when not many features are visible. Other changes concern the **filling of matrix S and vector e**; **I tried to avoid nested loops and most of the computations is now performed using the fast `cv::Matx` and `cv::Vec` objects instead of `cv::Mat_` matrices.** 

* In method `MakeTemplateCoarse` of the `Patchfinder`, the original code invoked first the creation of the warpping matrix and thereafter created the template using the precalculated warp. **The problem was, that in rare but not entirely unlikely occasions, the warping matrix could be singular because the tracker initiates the call (to `MakeTemplateCoarse` ) before it has managed to cherry-pick points**; thus, its is possible that a singular warping matrix can be passed to MakeTemplateCoarseCont thereby causing assertion trigger and subsequent halting of execution. **I just added an "if" that prevents the passing of the warping matrix if the "templateBad" flag has been raised by `CalcSearchLevelAndWarpMatrix`**. Of course, this is not a major change, but it really took me a while to spot this bugger and from what I see, it has survided up until now in the ETH version as well.  

* I created an interface class `Initializer` that defines the generic structure of a SLAM initialization method and added am **Essential matrix** based `Initializer` called `EssentialInit`. Thus PTAM can be initialized in a generic scene that does not necessarilly contains dominant plane. Of course, a dominant plane will always be computed based om the reconstructed points, but it will probably be a bad linear fit to the point cloud and will appear as a skewed plane.  


In general, there have been many such similar alterations (perhaps improvements) while converting the entire code and I really cant enumerate each and every one here... It should be noted that in the context of these changes, I really HAD to change the names of certain variables and functions because they were simply pointing at the wrong direction. For instance, a name "MakeTemplateSubPix" is a misleading name for a function that simply computes a Jacobian Gram-matrix accumulator (it was renamed to "PrepGNSubPixStep" which - in my opinion - is much closer to what the method actually does). However, I should point-out that most variable names were extremely well chosen and not only I kept them, but I was heavily influenced in adopting the exact same (or similar) naming conventions. 

I have also added plenty of comments explaining (matlab style) the actual algorithms behind the code (typically non-linear LS formulations in the Patchfinder.cpp, Calibrator.cpp, SmallBlurryImage.cpp and CalibImage.cpp). Now, my comments dominate the code (excluding the Bundle.cpp file which is sufficientlly commented); in several cases I was forced to erase existing comments because they were either misleading or very vague (perhaps they were added later-on?). So all in all, if you read comments, chances are 3/4 they are mine.

2. Other changes concern the software engineering side of things: For starters, no TooN, no libCVD and no GVars and Enter openCV:

I should note that with the exception of a Timer class from libCVD and the M-estimator header, every other file sustained major changes in order to accommodate the use of OpenCV structures. So, even if I did not change the specifics of an algorithm I would still have to make changes in order for all code to work exactly or approximately as thge original.    

Why OpenCV instead of the original dependencies?

Although TooN, libCVD and GVars are brilliant to say the least, they are not well documented online (except for the API reference of course) and examples simply do not exist. It has been much easier to go through the entire code line-by-line, rather than trying to write a working example just from the API reference; interestingly, even when certain functions had clear use in the original PTAM, I would still have to read the code in TooN or libCVD in order to gain insight on its specifics; such an example is "halfSample" which effectively is averaging decimation (I originally used pyramidal downsampling and (thereafter) simple resizing with interpolation as provided by OpenCV and discovered that Rosten's simple routine works better results for some reason and its just plain faster!) .

Thus, I ended-up lifting and modifying code such as the SE3, SO3, SO2, SE2 (TooN), GUI (GVars), FAST (libCVD) classes and the OpenGL interface to provide missing functionality using OpenCV Matx and Mat_ objects.
No need to mention some TooN code was a great mind opener, such as, for instance, the near-zero approximations to the Lie exponential, or the brilliant operator overloading schemes (I created mine, but the essential ideas came straight out of the original code).

Unlike other PTAM spawns (e.g., ETH PTAM or the recent ORBSLAM) I kept the OpenGL interface simply because it's awesome! It took a bit of effort to convert it to work with OpenCV matrices, let alone isolate the useful stuff from the rest of libCVD, but it was worth it! The same goes with GVars which I believe is much more useful than the standard .xml storage.


A few words about the functionlity of libCVD, GVars and TooN in the following source code directories:

a) GCVD : Contains some GCVD functionality (SO2, SO3, SE2, SE3) and the OpenGL interface (which I think is mostly Klein's work). There is an OpenGL window and a few custom developed controls which, in the world of Linux are very-very useful; I am very arrogant myself, but also wise to keep this stuff because they not only save time, but also have good taste in them! And I didnt even bother changing colors, ect. because they are simply nice as they are! 

b) FAST : Just the FAST stuff from libCVD with simple modifications for OpenCV Mat_<uchar> structures.

c) Persistence : This directory contains code that implements functionality of GVars. I converted the original GV3 gvar3 objects to PV3 and pvar3 to work with OpenCV Mat_ and Vec<P, int> types (I actually had to do a lot of template "tricks" to somehow entertain the fact that Mat_ objects dont come with a priori fixed size - all in all, I havent tested persistence with Mat_ objects, but it is unnecessary for now). I kept the GUI class nearly verbatim. I should note that perhaps it would be better to manage events from OpenGL, but it is unimportant. 


INSTALLATION
------------

** The necessary dependencies are OpenGL and OpenCV.  

** To compile, create a directory ``build``in the root directory of GPTAM, enter it with ``cd build`` and run ``cmake ..``  followed by a ``make``.

** I wasn't able to "automate" the OpenCV library settings, so I hard-coded the paths in the ``CMakeLists.txt`` which are the usuals: /usr/local/lib and usr/ local/include.  
  
RUNNING PTAM
------------
In the root directory you will find a fiile ``calibrator_settings.cfg`` containing the settings of the calibrator, including the initial parameters of the camera. The calibrator will use default settings if the respective file is not found. 

PTAM however requires the camera intrinsic and distortion parameters to be stored in a file named ``settings.cfg``. You will haveto fill this file with the intrinsic parameters provided by the calibrator. After saving the calibrated parameters, they should be stored a file named ``camera.cfg``. Simply rename this file to ``settings.cfg`` and you should be able to run ``gptam`` with the calibrated camera.  
