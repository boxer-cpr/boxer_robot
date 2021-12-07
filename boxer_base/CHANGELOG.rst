^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package boxer_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2021-12-07)
------------------
* Explicitly run the bridge scripts through bash
* Contributors: Chris Iverach-Brereton

0.1.4 (2021-12-06)
------------------
* Install the bridge scripts to the package's /share folder too; otherwise the install & start-bridge scripts won't work correctly
* Contributors: Chris Iverach-Brereton

0.1.3 (2021-12-02)
------------------
* Remove the roslint tests; we've removed the source code that would otherwise need linting
* Contributors: Chris Iverach-Brereton

0.1.2 (2021-12-02)
------------------
* Remove the ROS2 Foxy exec_depends temporarily so we can get the core package built. Foxy dependencies will need to be installed manually for now
* Contributors: Chris Iverach-Brereton

0.1.1 (2021-12-02)
------------------
* Add the ROS2 and clearpath_api dependencies for the bridge
* Sort the dependencies
* Only try sourcing the ROS2 API if the directory exists
* Dynamically load the correct ROS2 path when starting the bridges
* Remove a duplicate diagnostic analyzer node
* Remove an unsatisfiable dependency
* Add the IMU filter dependency
* Rename BOXER_SERIAL_NO to ROS_ROBOT_SERIAL_NO to match with incoming changes to the ISO
* Update the build instructions to remove unneeded dependencies, update the actual dependencies
* Teleop is now part of control.launch, so no need to start it separately
* Make all of the scripts executable.  In the export from the previous repository their permissions got mangled
* Contributors: Chris Iverach-Brereton

0.1.0 (2021-11-08)
------------------

* Initial commit of for Noetic
* Contributors: Chris Iverach-Brereton

0.0.10 (2020-08-24)
-------------------
* Updated camera framerates for hardware limitations
* Put parentheses around the print statements
* Contributors: Chris I-B, Dave Niewinski

0.0.9 (2020-01-16)
------------------
* Updated camera framerate
* Contributors: Dave Niewinski

0.0.8 (2019-08-28)
------------------

0.0.7 (2019-06-17)
------------------
* Temporarily removed point grey camera driver dep until it is fixed in the buildfarm
* Linting
* Contributors: Dave Niewinski

0.0.6 (2018-11-15)
------------------
* Updated camera config for spinnaker driver and synchronized triggering
* Added message_info_service
* Contributors: Dave Niewinski

0.0.5 (2018-11-07)
------------------
* Added IMU filter
* Contributors: Dave Niewinski

0.0.4 (2018-11-07)
------------------
* Updated dependencies
* Contributors: Dave Niewinski

0.0.3 (2018-11-05)
------------------

0.0.2 (2018-11-05)
------------------
* Dependencies
* Contributors: Dave Niewinski

0.0.1 (2018-10-31)
------------------
* Initial ish commit
* Contributors: Dave Niewinski
