^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vicon_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2017-08-04)
------------------
* Install vicon_driver headers as well as required boost libs
  This was causing compilation failures when installing to install.
* Fix metapackage directory structure to work with newer catkin tools
  Packages should not be nested.
  Make libvicon_driver (now vicon_driver) its own package so that vicon
  package can now depend on it. There is a silly hack needed to find the
  boost libraries needed by the ViconSDK.
  This should now be properly built using all versions of catkin tools.
* Contributors: Alex Spitzer

0.0.1 (2017-03-04)
------------------
