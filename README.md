TAS Group 10
============

1) Contributions

1.1) New Global and Local Planner

We integrated the SBPL Lattice Planner in order to achieve better performance than with the old planner. We also adjust the local planner and its corresponding parameters. To adjust the global planner especially to our car we created some motion primitives files with MATLAB.

Location: maskor_navigation package

This contribution was made by Christian Pfaffenzeller and Fabian Colapietro.


1.2) Monte Carlo Localization

Furthermore we implemented a automatical Initial Localization. Therefore we used the Monte Carlo Localizatinon and the car moves automatically around in order to fnd its current position.

Location: initial_localization package

This contribution was made by Peter Leidl and Dominik Mücke.


1.3) Velocity Control

We integrated an velocity control to adjust the velocity to the current situation on the track. So the car checks the distance to the next wall/obstacle in front of it and adjust its velocity. With that we especially achieve that the car slows down in front of bends.

Location: tas_autonomous_control/src/tas_autonomous_control_node.cpp

This contribution was made by Peter Leidl and Dominik Mücke.


1.4) Slalom

For the second task we chose the slalom parcour. For that task we recorded an new map with the pylons and added some corresponding waypoints through the parcour. In this task we profit by our SBPL Planner and its motion primitives.

Location: slalom package

This contribution was made by Peter Leidl, Dominik Mücke, Christian Pfaffenzeller and Fabian Colapietro.


1.5) State Machine

To organize the order of all processes we created some launch files (located in tas/launch). The usage of them are described in the Manual.txt.

Location: tas/launch

This contribution was made by Christian Pfaffenzeller and Dominik Mücke.



2) Usage

In the Manual.txt file its described how to how to use the code.

Location: tas/launch/Manual.txt.

