clc;clear all;close all
load('XUtraj_quad.mat')
N=40;
X0=zeros(6,1);
Xk=reshape(X,[6 N])
Xk=[X0,Xk]%41
Uk=reshape(U,[2 N])%40
%%
syms x [6,1]
syms u [2,1]
dxdt=fDyn(x,u)
A=jacobian(dxdt,x)
B=jacobian(dxdt,u)
Alin=[];Blin=[];
for k=1:N
    %(x0,u0),
    %
    x1=Xk(1,k);x2=Xk(2,k);x3=Xk(3,k);
    x4=Xk(4,k);x5=Xk(5,k);x6=Xk(6,k);
    u1=Uk(1,k);u2=Uk(2,k);
    Alin(:,:,k)=eval(A)
    Blin(:,:,k)=eval(B)
    %Alin(:,:,k)=double(subs(A,[x1;x2;x3;x4;x5;x6],Xk(:,k)))
end
Pf=10*diag([10;10;.1;.01;.01;.01]);
tsim=linspace(0,1,41);
[TRic,PRic]=ode45(@(t,P)Riccati(t,P,Alin,Blin),tsim,Pf);
size(PRic)
PRic=PRic(2:end,:);
tsim=0:0.025:1
X0=[-0.1;-0.1;0.1;1;1;0];

[T,X]=ode45(@(t,x)fDyn(x,control(t,x,Uk,Xk,Blin,PRic)),tsim,X0);
X=X';X=X(:);
%%
%close all
%figure(4)
%pause(3)
for i=2:N
    x=X(1+6*(i-1));
    y=X(2+6*(i-1));
    th=X(3+6*(i-1));
    u1=Uk(1,i);u2=Uk(2,i);
    plot(x+[-0.2*cos(th) 0.2*cos(th)],y+[-0.2*sin(th) 0.2*sin(th)],'bo-');hold on;grid on;
    quiver(x+[-0.2*cos(th) 0.2*cos(th)],y+[-0.2*sin(th) 0.2*sin(th)],-sin(th)*[u1 u2],cos(th)*[u1 u2]);hold on;grid on;
    %plot([0.5 0.5],[-0.2 10],'k','LineWidth',2);
    plot([0.5 0.5],[-10 0.2],'k','LineWidth',2)
    axis([-1.5 1.5 -1.5 1.5])
    drawnow;
    pause(1.04);
    clf;
end
%%
function u=control(t,x,Uk,Xk,Bk,PRic)
    k=floor(t/0.025)+1;
    k=min(k,40);
    P=PRic(41-k,:);
    P=reshape(P,[6 6]);
    R=0.001*eye(2);
    K=inv(R)*Bk(:,:,k)'*P
    u=Uk(:,k)-K*(x-Xk(:,k));
end
%%
function dxdt=fDyn(x,u)
    % x,y,th,dx,dy,dth
    m=1.03;
    I=1.06;
    r=0.39;
    g=9.81;
    dxdt=[x(4:6);
        -(u(1)+u(2))*sin(x(3))/m;
        ((u(1)+u(2))*cos(x(3))-m*g)/m;
        r*(u(1)-u(2))/I];
end
function dPdt=Riccati(t,P,Ak,Bk)
    Q=diag([100;100;1000;1;1;1]);
    R=0.001*eye(2);
    Rinv=inv(R);
    P=reshape(P,[6 6]);
    k=floor(t/0.025);
    k=max(40-k,1);
    A=Ak(:,:,k);
    B=Bk(:,:,k);
    dPdt=A'*P+P*A-P*B*Rinv*B'*P+Q;
    dPdt=dPdt(:);
end