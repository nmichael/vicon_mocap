^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vicon_mocap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2017-08-04)
------------------
* Merge branch 'feature/covariance' into develop
* Fix metapackage directory structure to work with newer catkin tools
  Packages should not be nested.
  Make libvicon_driver (now vicon_driver) its own package so that vicon
  package can now depend on it. There is a silly hack needed to find the
  boost libraries needed by the ViconSDK.
  This should now be properly built using all versions of catkin tools.
* Contributors: Alex Spitzer, John Yao

0.0.1 (2017-03-04)
------------------
