6.837 Lab 3
===========

Running code
------------
-To compile, cd to build and type "make install". Then to run the file, cd to starter3 and type the following:

"[FILE] [SYSTEM] [STEP SIZE] [INTEGRATOR] {PARTICLE i TO DISPLAY ITS SPRINGS}"

[FILE] = inst/a3

[SYSTEM] = 's' for Simple System, 'p' for Pendulum System, 'c' for Cloth System

[STEP SIZE] = a float signifying step size for integrator (my testing was all done with 0.04)

[INEGRATOR] = 'e' for Euler, 't' for Trapezoidal, 'r' for Runge Kutta

{PARTICLE i TO DISPLAY ITS SPRINGS} = an int signifying particle i whose springs should be drawn instead of the wireframe. This is an OPTIONAL parameter. If it's not provided, you will be able to toggle through the wireframe. If it is provided, it will always display the springs connected to particle i. 

While the program is running you can:

-Press 'w' to turn the wireframe on or off. Only works if optional command line paramter (particle i to display its springs) was not inputted

-Press 'm' to have the cloth move. This is only for the cloth system. It will move forward for a few seconds, and then begin to move backwards. The time to move forward/back is a bit long (though no longer than a minute) to be able to move an interesting amount of the cloth.



Collaboration
-------------
-Zachary Sather. Mostly conceptual questions. Some help with memory leak errors.


References
----------
-Used several standard sites (like stack overflow) for some OpenGL tools (like drawing lines)


Problems
--------
-None that I know of.


EC
--
-None


Comments
--------
-I enjoyed the assignment, even though it was a tough one. I had a memory leak error that took me a while to find. I would add a reminder for people to be careful with standard libraries like vector. I think it was a common issue with students. My main suggestion for improving the pset is providing a list of requirements at the end of the handout. Many requirements were buried within text, and a concise list at the end would have made checking that everything was completed much easier.
