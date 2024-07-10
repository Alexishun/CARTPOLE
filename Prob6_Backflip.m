%% Trajectory Optimization masa resorte forzado
clc;clear all;close all
% Dinamica
% dx=f(x,u)-> (x(k+1)-x(k))/Dt=f(x,u)
% dx=f(x,u)->  x(k+1)=x(k)+Dt*f(x(k),u(k))
% f(x,u)=[x2;-b/(m*l^2)*x2-g/l*sin(x1)+u/(m*l^2)]
% 60 discretizaciones
% abs(u)<=100
% x1=x0+Dt*u0
% x2=x1+Dt*u1
% x3=x2+Dt*u2
% x4=x3+Dt*u3
%...
% x10=x9+Dt*u9
% z=[x1;x2;...;x10;u0;u1;...;u9] %10*6+10*2=80
N=40;
tf=1;
Dt=tf/N;
n=6;%cantidad de estados
m=2;%cantidad de inputs
Z0=zeros(N*(m+n),1);
options = optimoptions(@fmincon,'MaxFunctionEvaluations',120000)
[zmin,valmin]=fmincon(@(x)Objet(x),Z0,[],[],[],[],[],[],@(x)restr(x),options)
X=zmin(1:N*n);
U=zmin(N*n+1:end);
save('XUtraj_quad','X','U')
figure(1)
plot(X(1:n:end),'o-');hold on;% 1 7 13 19 ... posicion x
plot(X(4:n:end),'o-');hold on;% 1 7 13 19 ... velocidad x
%k_plot=1:10;
%plot(k_plot,(k_plot*Dt)-(k_plot*Dt).^2,'o-')% 1 3 5 7
legend('pos x','vel x')
title('quadrotor x')
figure(2)
plot(X(3:n:end),'o-');hold on;% 1 7 13 19 ... posicion x
plot(X(6:n:end),'o-');hold on;% 1 7 13 19 ... velocidad x
%k_plot=1:10;
%plot(k_plot,(k_plot*Dt)-(k_plot*Dt).^2,'o-')% 1 3 5 7
legend('pos th','vel th')
title('quadrotor th')
figure(3)
plot(U(1:2:end),'o-');hold on;
plot(U(2:2:end),'o-');
legend('u1','u2')
%%
%close all
figure(4)
pause(3)
for i=1:N
    x=X(1+6*(i-1));
    y=X(2+6*(i-1));
    th=X(3+6*(i-1));
    plot(x+[-0.2*cos(th) 0.2*cos(th)],y+[-0.2*sin(th) 0.2*sin(th)],'ro-');hold on;grid on;
    %plot([0.5 0.5],[-0.2 10],'k','LineWidth',2);
    plot([0.5 0.5],[-10 0.2],'k','LineWidth',2)
    axis([-1.5 1.5 -1.5 1.5])
    drawnow;
    %pause(0.1);
    %clf;
end
%%
% Uref=U;
% tsim=0:0.025:1
% X0=[0.0;0.0;-0.0;-0;-0;0];
% [T,X]=ode45(@(t,x)fDyn(x,control(t,x,Uref)),tsim,X0);
% %X=X';X=X(:);
% %%
% %close all
% %figure(4)
% %pause(3)
% for i=2:N
%     x=X(1+6*(i-1));
%     y=X(2+6*(i-1));
%     th=X(3+6*(i-1));
%     plot(x+[-0.2*cos(th) 0.2*cos(th)],y+[-0.2*sin(th) 0.2*sin(th)],'bo-');hold on;grid on;
%     %plot([0.5 0.5],[-0.2 10],'k','LineWidth',2);
%     plot([0.5 0.5],[-10 0.2],'k','LineWidth',2)
%     axis([-1.5 1.5 -1.5 1.5])
%     drawnow;
%     %pause(0.1);
%     %clf;
% end

%%
function u=control(t,x,Uref)
    k=floor(t/0.025)+1;
    k=min(k,40);
    u1=Uref(2*k-1);
    u2=Uref(2*k);
    u=[u1;u2];
    %u=Uref(t)-K*(x-xref(t))
end
function J=Objet(z)
    n=6;
    m=2;
    N=40;
    X=z(1:N*n); % 8 estados en total
    U=z(N*n+1:end); %4 entradas de control
    Q=1000;
    R=0.5;
    % J1=0;
    % for k=1:60
    %     xkref=(k*Dt)-(k*Dt)^2;
    %     J1=J1+(X(2*k-1)-xkref)'*Q*(X(2*k-1)-xkref);
    % end
    Qlinx=1;
    Qliny=1;
    Qf=diag([100;1;1000;10000;1;1]);
    R=0.00001*eye(2);
    J1=(X(end-5:end)-[1;0;2*pi;0;0;0])'*Qf*(X(end-5:end)-[1;0;2*pi;0;0;0]);% ultimos 6 valores
    J2cost=0;
    for k=1:N
        J2cost=J2cost+U(2*k-1:2*k)'*R*U(2*k-1:2*k);
    end
    for k=1:N
        J1=J1+(X(6*k-5)-k/N)'*Qlinx*(X(6*k-5)-k/N);
    end
    %J2cost=U'*R*U;
    J=J1+J2cost;
end
function [c,ceq]=restr(z) %c<=0  ceq==0
    x0=zeros(6,1);
    tf=1;
    N=40;
    n=6;
    X=z(1:N*n);
    U=z(N*n+1:end);
    Dt=tf/N;
    % u<=1; u>=-1 -> -1-u<=0
    umax=70;
    umin=-30;
    c=zeros(4*N,1);
    for k=1:N
        %c([4*k-3:4*k],1)=[U(2*k-1:2*k)-umax;-U(2*k-1:2*k)+umin];
    end
    for k=17:23
        %c=[c;X(2+6*(k-1))+0.2*sin(X(3+6*(k-1)))+0.2];%cola adelante
        %c=[c;X(2+6*(k-1))-0.2*sin(X(3+6*(k-1)))+0.2];%cola atras
        c=[c;-(X(2+6*(k-1))+0.2*sin(X(3+6*(k-1))))+0.2];%cola adelante
        c=[c;-(X(2+6*(k-1))-0.2*sin(X(3+6*(k-1))))+0.2];%cola atras
    end
    % for k=1:N
    %     c=[c;X(2+6*(k-1))+0.2*sin(X(3+6*(k-1)))-0];%cola adelante
    %     c=[c;X(2+6*(k-1))-0.2*sin(X(3+6*(k-1)))-0];%cola atras
    % end
    %x(k+1)=x(k)+Dt*f(x(k),u(k))
    %x_1=x_0+Dt*f(x_0,u_0) 2 ecuaciones
    ceqDYN=zeros(n*N,1);
    ceqDYN(1:6,1)=[-X(1:6)+x0+Dt*fDyn(x0,U(1:2))];
    for k=2:N
        ceqDYN([6*k-5:6*k],1)=-X([6*k-5:6*k])+X([6*k-11:6*k-6])+Dt*fDyn(X([6*k-11:6*k-6]),U(2*k-1:2*k));
    end

    ceqADD=[];
    ceq=[ceqDYN;ceqADD];

end

function dxdt=fDyn(x,u)
    % x,y,th,dx,dy,dth
    m=1;
    I=1;
    r=0.4;
    g=9.81;
    dxdt=[x(4:6);
        -(u(1)+u(2))*sin(x(3))/m;
        ((u(1)+u(2))*cos(x(3))-m*g)/m;
        r*(u(1)-u(2))/I];
end