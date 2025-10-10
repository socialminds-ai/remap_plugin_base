^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package remap_plugin_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2025-10-10)
------------------
* Fix ament_auto warning about headers install destination
* Contributors: Noel Jimenez

0.3.0 (2025-06-18)
------------------
* overriding default node options
  we are providing custom node options, which are basically
  the same as the default ones except for the fact that we
  remove the default node name (the one passed via command line
  arguments). This way, the plugin node name is actually initialized
  with the one passed as an argument to the node constructor.
* README
* Contributors: Lorenzo Ferrini

0.2.0 (2025-04-07)
------------------
* adding functions to revise kb
* supporting both regions and entities spatial relationships
* added function to process spatial relationships
* Contributors: Lorenzo Ferrini

0.1.0 (2025-02-24)
------------------
* making linters happy
* add dep on TBB
* we actually need libopenvdb-dev
* add missing dep on openvdb and vbd2pc is actually needed
* fixed CMakeLists + removed useless vdb2pc dependency
* removed reg_of_space_server dependency
* README.md
* included semantic plugin base class
* separated regions register
* making linters happy
* initial commit
* Contributors: Lorenzo Ferrini, Séverin Lemaignan
