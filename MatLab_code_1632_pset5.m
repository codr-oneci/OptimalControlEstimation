%16.32 Assignment 5 Problem 2
Q=[1, 0; 0, 1]; %state cost Rxx
R=0.01; %control cost Ruu
N=[0;0]; %mixed component of const x.T*N*u
A=[1, 1; 1, 2];
B=[1;0];
C=[1, 0];
D=0;

%solve LQR problem for part 1
sys=ss(A,B,C,D);
[K,S,e] = lqr(sys,Q,R,N);
sys_lqr=ss((A-B*K),B,C,D);
figure(1);
step(sys_lqr)
%lqr with modified control law u=N*r-K*x, where N=0.5 from final limit
%theorem
sys_lqr_N=ss((A-B*K),B*0.5,C,D);
figure(2);
step(sys_lqr_N)


%part 3 of problem 2
Ap=[1,1,0;1,2,0;-1,0,0];
Bp=[1;0;0];
Cp=[1,0,0];

k1=K(1);k2=K(2); %take previous LQR steady state values
ki=8 %for this part, manually tune the integral term ki

Api=Ap-Bp*[k1,k2,ki]
Bpi=[1;0;0];
Cpi=Cp;
sys_lqr_pi=ss(Api,Bpi,Cpi,0);
figure(3);
step(sys_lqr_pi)

% problem 3 solution
disp("Convert transfer function to state space for Problem 3")
b=[-1 3];
a=[-1 1 12];
[A,B,C,D]=tf2ss(b,a)
sys = ss(A,B,C,D);
nx = 2;    %Number of states
ny = 1;    %Number of outputs
Qn =[3 0;0 3];
Rn = 5;
Ruu = 1;
Rxx=4*eye(nx);
QXU = blkdiag(Rxx,Ruu);
QWV = blkdiag(Qn,Rn);
QI = eye(ny);
KLQG = lqg(sys,QXU,QWV) %Linear Quadratic Gaussian Gain
%KLQG1 = lqg(sys,QXU,QWV,QI,'1dof')
%check the step response of this controller using the LQG feedback contolled system
%sys_LQG= ss(A-KLQG*C,[B KLQG],C, 0*[B KLQG]);
figure(4);
step(KLQG)

% problem 4 solution
A=[0,1;-6,-5];
B=[0;1];
C=[1,1];
D=0;
sys=ss(A,B,C,D);
Rxx=transpose(C)*C;
Ruu=0.01
[K,S,e]=lqr(sys,Rxx,Ruu);
disp("K,e and S for the LQR controller from problem 4")
disp(K)
disp(e)
disp(S)


sys_lqr=ss((A-B*K),B,C,D);
%problem 4 setting up initial system conditions and observying the system
%response
figure(5);
x0 = [1.0 0.0];
[y,t,x]=initial(sys_lqr,x0);
xlabel('time')
ylabel('x')
figure(6)
plot(t,y)
xlabel('time')
ylabel('y')
title("Problem 4 LQR control amplitude vs time")
xlabel('Times [s]')

%plot(t,x)
u=transpose(-K*transpose(x));
figure(7)
plot(t,u)
title('LQR Problem 4 control input')


%problem 4 parts 3 and 4 solution
%define system with modified observability matrix
A=[0,1;-6,-5];
B=[0;1];
C=[1,-1];
D=0;
sys2=ss(A,B,C,D);
Rxx=transpose(C)*C;
Ruu=0.01
[K,S,e]=lqr(sys2,Rxx,Ruu)
disp("K,e and S for the LQR controller from problem 4")
disp(K)
disp(e)
disp(S)


sys_lqr_m=ss((A-B*K),B,C,D);
%problem 4 parts 3 and 4 setting up initial system conditions and observying the system
%response
figure(8)
x0 = [1.0, 0.0];
[y,t,x]=initial(sys_lqr_m,x0);

figure(9)
plot(t,y)
title("Problem 4 LQR control amplitude vs time")
xlabel('Times [s]')

%plot(t,x)
u=transpose(-K*transpose(x));
figure(10)
plot(t,u)
xlabel('time [s]')
ylabel('contol input (u)')
title('LQR Problem 4 control input')










