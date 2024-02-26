function j = z_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
w5=1e4;
 %Implement your code here
%% Construct the prediction matrix
[Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix_nonzero(K,dt,p_0,v_0,a_0,pt,vt,at);

%% Construct the optimization problem
Bv_1=Bv+vt;
Ba_1=Ba+at;
H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
F = [w1*Bp'*Tp+w2*Bv'*Tv+w3*Ba'*Ta,zeros(1,K)];
A=[Tv, zeros(K);-Tv, zeros(K);Ta, zeros(K);-Ta, -eye(K);eye(K),zeros(K,K); -eye(K),zeros(K,K);zeros(K,K),-eye(K)];
b=[6*ones(K,1)-Bv_1;ones(K,1)+Bv_1;3*ones(K,1)-Ba_1;ones(K,1)+Ba_1;2*ones(2*K,1);zeros(K,1)];
%% Solve the optimization problem
J = quadprog(H,F,A,b);
j=J(1);
end