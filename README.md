# VTFS - A Visuo-Tactile-Force-Servo Controller

This is a C++ based serial manipulator servo controller. The feedback modality 
includes joint angle, visual, tactile and force. Currently
this controller has been tested on the Kuka lwr and iCub.

- This software has been developed with the generous support of the
  - CITEC (Cognitive Interaction Technology - Center of Excellence),

               http://www.cit-ec.de/

  - DFG SPP 1527

               http://ipvs.informatik.uni-stuttgart.de/mlr/spp-wordpress/projekte/projects-entries/?wcteid=16

  - Honda Research Institute, Europe

               http://www.honda-ri.de/tiki-index.php?page=Welcome



The author would also like to thank 
Dr. Robert Haschke and Prof. Dr. Helge Ritter of the 
Neuroinformatics Group at Bielefeld University for their 
many helpful comments and insightful discussions..

http://www.ni.techfak.uni-bielefeld.de

- Literature

  - http://www.roboticsproceedings.org/rss09/p45.html
  - http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6696703&tag=1

- How to get the software

  - Install the git toolchain on you computer. 
  - You can use e.g. this comand to check the current version of the software into a directory called cbf:

           git clone https://github.com/liqiang1980/VTFS.git

- Requirements

  - KDL, for KDL based inverse and forward kinematics
  - CBF, controller basis framework developed in the Bielefeld University.


- Installation

See the INSTALL file for information on how to build and 
install the software

- Author

Qiang Li (qiang.li.mail@gmail.com)