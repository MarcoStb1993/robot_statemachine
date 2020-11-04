^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rsm_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2020-11-04)
------------------
* Put GetRobotPose service back in
* Exploration Goal Status is now published as a topic to prevent service deadlocks
* Removed failed goal services and added a publisher
* Added GoalCompleted service to give more detailed feedback over navigation completed status
* Added prefix to state name to identify autonomy behavior
* Added services for handling completed navigation goals to remove the logic from the navigation plugin
* Removed double maintainer tags
* Contributors: MarcoStb1993

1.1.3 (2019-08-29)
------------------
* Added all dependencies to CMakeLists and package.xml, that were missing previously
* Contributors: MarcoStb1993

1.1.2 (2019-08-28)
------------------
* Added necessary dependency on geometry_msgs to package.xml
* Contributors: MarcoStb1993

1.1.1 (2019-08-05)
------------------
* added changelogs
* Changed package names, this time for real
* Contributors: MarcoStb1993
