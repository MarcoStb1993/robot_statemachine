^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rsm_additions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2020-11-04)
------------------
* Added parameter for navigation behavior on idle timer callback and
  gazeboToTf changed to ground plane
* Fixes for navigation idle timer
* Get Odom tf directly from gazebo instead of EKF
* Node to retrieve TF directly from gazebo
* Added reverse movement as unstuck behavior to navigation state
* Added params for idle timer and pose tolerance
* Compare pose to identify idling and state transition on goal obsolete
* Replaced simulated kinect with Intel RealSense to adapt to husky
  simulator melodic
* Clipped Kinect range to ignore laser scanner
* Now sets goal as finished only after completing mapping or routine state
* Fixed Kinect control joint name
* Exploration Goal Status is now published as a topic to prevent service deadlocks
* Removed failed goal services and added a publisher
* Added GoalCompleted service to give more detailed feedback over navigation completed status
* Added prefix to state name to identify autonomy behavior
* moved goalObsolete service to Additions and exploration completed goal out of navigation completed service
* Added navigation completed service call to goal obsolete callback
* Only publish exploration goals when explore_lite is used
* Additions Service Provider only subscribes and listens to services and topics for active plugins
* Added tutorial details for creating Calculate Goal and Navigation plugins
* Added services for handling completed navigation goals to remove the logic from the navigation plugin
* Link corrections in README
* Moved logic regarding explore_lite to AdditionsServiceProvider from ServiceProvider where they should belong
* Removed double maintainer tags
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
