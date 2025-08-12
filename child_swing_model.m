%% Model of 4DOF child on a swing
%% Example in IEEECDC 2025 paper
%% "A Hybrid Orbital Stabilizer With Guaranteed Basin of Attraction for
%% Mechanical Systems With Underactuation Degree One"
%% Authors: L.D. Navarro and M. Maggiore
% Copyright 2025, Luiz Dias Navarro and Manfredi Maggiore.
% For academic use only. Please cite the IEEE CDC 2025 paper if used.


m1=0;
m2=0.3;
m3=0.15;
m4=0.55;
l1=4;
l2=1;
l3=1;
l4=2;
g=9.8;

syms q1 q2 q3 q4 q1dot q2dot q3dot q4dot real

q=[q1;q2;q3;q4];
qdot=[q1dot;q2dot;q3dot;q4dot];

% Define centres of mass of each link
rc1=l1*[-sin(q1);cos(q1)];
rc2=rc1+l2*[-sin(q1+q2);cos(q1+q2)];
rc3=rc2+l3*[-sin(q1+q2+q3);cos(q1+q2+q3)];
rc4=rc1+l4*[-sin(q1+q4);cos(q1+q4)];

% Compute time derivatives of centres of mass
rc1dot=jacobian(rc1,q)*qdot;
rc2dot=jacobian(rc2,q)*qdot;
rc3dot=jacobian(rc3,q)*qdot;
rc4dot=jacobian(rc4,q)*qdot;

% Define the total kinetic energy of the robot
K=1/2*(m1*(rc1dot'*rc1dot)+m2*(rc2dot'*rc2dot)+m3*(rc3dot'*rc3dot)+m4*(rc4dot'*rc4dot));
K=simplify(K);

% Extract the square symmetric matrix of the kinetic energy
D=simplify(hessian(K,qdot));

% Computation of matrix C(q,qdot) of pinned robot
C = sym(zeros(4,4));
for i=1:4
    for j=1:4
        for k=1:4
            C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*qdot(i);
        end
    end
end

% Define the potential energy of the pinnedrobot
P = g*[0,1]*(m1*rc1+m2*rc2+m3*rc3+m4*rc4);

% Computation of gradient of the potential for the pinned robot
dP = jacobian(P,q)';

B = [zeros(1,3);eye(3)];
sys.D = matlabFunction(D,'Vars',{q});
sys.C = matlabFunction(C,'Vars',{q;qdot});
sys.dP= matlabFunction(dP,'Vars',{q});
sys.B = @(q)    B;

sys.ddq= @(q, dq, u) sys.D(q)\(sys.B(q)*u-sys.C(q,dq)*dq-sys.dP(q));
sys.xdot= @(x, u) [x(5:8);sys.ddq(x(1:4), x(5:8), u)];

sys.Bp = @(q)[1,0,0,0];

sys.rc1 = matlabFunction(rc1, 'Vars', {q});
sys.rc2 = matlabFunction(rc2, 'Vars', {q});
sys.rc3 = matlabFunction(rc3, 'Vars', {q});
sys.rc4 = matlabFunction(rc4, 'Vars', {q});


disp('Mass matrix D(q):');
disp(D);

disp('Coriolis matrix C(q,qdot):');
disp(C);

disp('Gravity vector nabla P(q):');
disp(dP);

disp('Input matrix B:');
disp(B);