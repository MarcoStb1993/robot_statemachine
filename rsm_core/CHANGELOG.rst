^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rsm_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2020-11-24)
------------------

1.2.0 (2020-11-04)
------------------
* Added parameter for navigation behavior on idle timer callback and
  gazeboToTf changed to ground plane
* Replaced simulated kinect with Intel RealSense to adapt to husky
  simulator melodic
* Now starts exploration with mapping state
* Exploration Goal Status is now published as a topic to prevent service deadlocks
* Removed failed goal services and added a publisher
* Added GoalCompleted service to give more detailed feedback over navigation completed status
* Added prefix to state name to identify autonomy behavior
* moved goalObsolete service to Additions and exploration completed goal out of navigation completed service
* Fix wrong error message on service failure
* Added class and state diagram to documentation
* Added tutorial details for creating Calculate Goal and Navigation plugins
* Added services for handling completed navigation goals to remove the logic from the navigation plugin
* Moved logic regarding explore_lite to AdditionsServiceProvider from ServiceProvider where they should belong
* Removed double maintainer tags
* Updated README with joystick listener addition
* Added joystick topic for teleoperation interrupt
* Properly reset shared_ptr to prevent errors on close
* Contributors: Marco Steinbrink, MarcoStb1993, marco

1.1.3 (2019-08-29)
------------------
* Added all dependencies to CMakeLists and package.xml, that were missing previously
* Contributors: MarcoStb1993

1.1.2 (2019-08-28)
------------------

1.1.1 (2019-08-05)
------------------
* added changelogs
* Fixed some dependencies for the new names
* Changed package names, this time for real
* Contributors: MarcoStb1993
