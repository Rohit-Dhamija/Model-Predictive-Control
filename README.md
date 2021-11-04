# Model-Predictive-Control(MPC) for Path Tracking
This repository contains the implementation of Model-Predictive-Control(MPC) algorithm for path tracking on [Toyota Prius simulator](https://github.com/osrf/car_demo) using python, CasADi ipopt solver and [ROS](https://www.ros.org/) framework. The vehicle velocity is one of the state variables in the vehicle model used and hence, the Velocity Control is achieved by adding its cost in the MPC cost function. In order to supply a variable target velocity at different points in the path, update parameters(P) accordingly. <br />
References:<br />
[Model predictive control: past, present and future](https://www.sciencedirect.com/science/article/pii/0019057894900957)<br />
[An Introduction to Model-based Predictive Control (MPC) by Stanislaw H. Żak](https://www.researchgate.net/profile/Mohamed_Mourad_Lafifi/post/Model-Predictive-Control-examples/attachment/60202ac761fb570001029f61/AS%3A988637009301508%401612720839656/download/An+Introduction+to+Model-based+Predictive+Control+%28MPC%29.pdf)<br />
[Model Predictive Control System Design and Implementation Using MATLAB® by Liuping Wang](https://books.google.co.in/books?hl=en&lr=&id=vcc_AAAAQBAJ&oi=fnd&pg=PR27&dq=Model+Predictive+Control+System+Design+and+Implementation+Using+MATLAB+%C2%AE&ots=ivywSbpPYl&sig=kVciiao6kw3XqNlW9ohULHtdpA0#v=onepage&q=Model%20Predictive%20Control%20System%20Design%20and%20Implementation%20Using%20MATLAB%20%C2%AE&f=false)<br />
[Implementation of Model Predictive Control (MPC) and Moving Horizon Estimation (MHE) on Matlab](https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi/blob/master/workshop_github/MPC_MHE_slides.pdf)<br />

## Visualisations

![mpc](/Visualisations/mpc_multipleShooting_pathTracking_carDemo.gif)	
( The Target Velocity is set constant throughout the path as 10 m/s (36 kmph) )

