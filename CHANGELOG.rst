^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vicon_mocap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adding missing dependency library binary
* Fixes construction of tf broadcaster before init. Copies identity yaml. Copies debug lib for OSX
* Adding tf2 broadcaster of vicon pose
* Ensure vicon msgs are generated prior to building vicon_odom
* Defaulting to new API on OSX and old API otherwise
* Merge branch 'develop' of github.com:nmichael/vicon_mocap into develop
* Installing libvicon libraries
* Fixing string list append to strcat
* Looking for flag to check for the yaml-cpp API type. AFAIK, yaml-cpp does not provide a version macro so this is a manual flag option that defaults to the old API (std on ubuntu).
* Update README.md
* Restructure, hydro support, move to cmake build
* Fix major bug in 784813b, need to get a frame before using GetFrameRate
  Also updating the frame_time calculation so that the frame times are
  close to actual clock time with the jitter filtered out.
* Updates the readme
  Calibration routine now can be run from any location
* Adds compilation to the README
* Use framerate from Vicon to initialize frame dt + small README updates
* Even more README updates
* More README updates
* Updates the README
* Update the README and adds some flexibility to the calibrate launch file
* Fixing the calib pose calculation and usage
* Updating vicon_sdk to v1.3.0 + related changes
  Note: Even though the libraries for MacOSX are included, I have not
  tested this on MacOSX yet.
* Adding the missing vicon_sdk libraries
* Minor cleanup
* Small change in the way calib data is stored (inverse of previous)
* Fixes and improvements in the calibration process
  Now it allows the user to recalib even though a calib file is already
  present, the calibrate node will give the incremental transformation but
  the main driver calculates the correct transformation to store.
* Disable unlabeled markers by default in the ROS driver
* Adding libvicon_driver and its IPC and ROS interfaces
* Contributors: Kartik Mohta, Nathan Michael, justinthomas
