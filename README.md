# MPC_project
MPC controller for a quadcopter

The project is composed of the files MPC_Control_x/y/z/yaw.m that each implements an independant MPC controller.
The file Quad.m is a quadcopter object with all the parameters and method defining a quadcopter.
The file project.m implements the ToDo's described in the project definition.

To run the project, first run the Setup part of project.m, then you can run each separate part. You will need to change the file setup.m with your own path to the matlab folder and the casadi folder.

To use the optimizers needed for this project, please install the tools described here: https://moodle.epfl.ch/pluginfile.php/2712483/mod_resource/content/2/setup.pdf
