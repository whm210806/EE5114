Run EE5114_CA1.m

The function of the transformational matrix between the ECEF frame to the local NED frame is in R_ne.m

The function of the transformational matrix between the body frame to the local NED frame is in Body_NED.m

EE5114_CA1.m has four outputs: x, x1, x2, x3.

x: Output of linear bias.
x1: Output of constant bias.
x2: Output of correction every five times.
x3: Output of changing noise.

Run EE5114_CA1_Sigmoid.m for the EKF estimation with the sigmoid bias function.

The function of the sigmoid function is in sigmoid_basis.m

EE5114_CA1_Sigmoid.m has two outputs: x, x1;

x: Output of sigmoid bias.
x1: Output of constant bias.

