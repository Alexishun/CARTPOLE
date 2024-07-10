clc;clear all;close all;
%% Factor de reduccion del motor
% Reducción interna
n1=131;
n2=65/16;
n3=2*pi*0.5/65;
kr=n1*n2*n3;
%% Cinemática Directa
syms q1 q2 dq1 dq2 u
Cim=[0 q1 31.5e-3 -pi/2;
    q2 84e-3 202e-3 0];
A10=Mtran(0,q1,31.5e-3,-pi/2);
A21=Mtran(q2,84e-3,202e-3,0);
%% Matrices de Transformacion
T10=A10;
T20=A10*A21;
%% Coordenadas Centro de masa - eslabones
pl11=[-17.69;-0.07;-16.74]*1e-3;
pl22=[-127.02;0;-23.50]*1e-3;

pl10=T10*[pl11;1];pl10=pl10(1:3);
pl20=T20*[pl22;1];pl20=pl20(1:3);
%% Coordenadas Centro de masa - motores
pm10=[-11.48;11.24;-151.11]*1e-3;
%% Jacobianos
% Jacobiano eslabones
z00=[0;0;1];z10=T10(1:3,3);
Jvl1=[z00,zeros(3,1)];
Jvl2=[z00,cross(z10,pl20-T10(1:3,4))];
Jwl1=zeros(3,2);
Jwl2=[zeros(3,1),z10];

% Jacobiano motores
Jvm=zeros(3,2);
Jwm=[kr*[0;-1;0],zeros(3,1)];

%% Matrices de Inercia
% Eslabon 1
ml1=369.94e-3;
Il11=[1119063.15,130.92,-59196.70;
    130.92,1054693.08,1452.42;
    -59196.70,1452.42,271275.64]*1e-9;

% Eslabon 2
ml2=161.58e-3;
Il22=[260709.64,66.63,284714.62;
    66.63,1231711.01,78.70;
    284714.62,78.7,982253.49]*1e-9;

% Motor
mm=96.44e-3;
Im0=[33368.23, -1166.60, -1.27;
    -1166.60, 18618.52, 6.33;
    -1.27, 6.33, 33643.00]*1e-9;
%% Cálculo D
R1=T10(1:3,1:3);
R2=T20(1:3,1:3);
Rm=diag([1,1,1]);

Dl1v=ml1*Jvl1.'*Jvl1;
Dl1w=Jwl1.'*R1*Il11*R1.'*Jwl1;
Dl2v=ml2*Jvl2.'*Jvl2;
Dl2w=Jwl2.'*R2*Il22*R2.'*Jwl2;
Dmv=mm*Jvm.'*Jvm;
Dmw=Jwm.'*Rm*Im0*Rm.'*Jwm;
D=Dl1v+Dl1w+Dl2v+Dl2w+Dmv+Dmw;
disp('Matriz de inercia D:');
disp(D);
%% Matriz de Christoffel
Gamma = christoffel_symbols(D, [q1; q2]);
disp('Términos de Christoffel:');
disp(Gamma);
C=Gamma(:,:,1)*dq1+Gamma(:,:,2)*dq2;
%% Matriz de Gravedad
q=[q1;q2];
g=[-9.81;0;0];
Ptl1=-ml1*g.'*pl10;
Ptl2=-ml2*g.'*pl20;
Ptm1=-mm*g.'*pm10;
Pt=Ptl1+Ptl2+Ptm1;
G=jacobian(Pt,q);G=G.';
%% Fuerzas Generalizadas
dq=[dq1;dq2];
T=[u;0];
Bm=0;
Fv=[Bm*kr^2 0;0 0];
E=T-Fv*dq;

%% Ecuacion Diferencial
dqq=eval(simplify((D^(-1))*(E-C*dq-G)));
syms v
u=solve(dqq(1)-v,u);
dqq=simplify(eval(dqq))

%% Funciones
function A=Mtran(th,d,a,alp)
    A=[cos(th) -sin(th)*cos(alp) sin(th)*sin(alp) a*cos(th);
       sin(th) cos(th)*cos(alp) -cos(th)*sin(alp) a*sin(th);
       0 sin(alp) cos(alp) d;
       0 0 0 1];
end
% Función para calcular los términos de Christoffel
function Gamma = christoffel_symbols(D, q)
    n = length(q); % Número de coordenadas generalizadas
    Gamma = sym('Gamma', [n, n, n]); % Inicializar símbolos de Christoffel

    % Calcular los términos de Christoffel
    for i = 1:n
        for j = 1:n
            for k = 1:n
                Gamma(i, j, k) = 0.5 * (diff(D(i, j), q(k)) + diff(D(i, k), q(j)) - diff(D(j, k), q(i)));
            end
        end
    end
end
