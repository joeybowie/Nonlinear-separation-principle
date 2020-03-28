clc;
clear;
close all;
%% Model parameters
global M m b L I g
M = 1.378;   % Mass of cart                      [kg]
m = 0.051;   % Mass of pendulum                  [kg]
b = 12.98;   % Coefficient of friction for cart  [N/ms]
L = 4*0.325; % Length to pendulum center of mass [m]
I = 0.006;   % Moment of inertia of the pendulum [kgm^2]
g = 9.81;    % Acceleration due to gravity       [m/s^2]
%% Gains for control law
global kp ki kl
kp = 25;
ki = 0.25;
kl = 1;
%% Initial conditions: x = [theta theta_dot x x_dot]'
x0 = [deg2rad(65) -0.05 0 0]'; 
%% High-gain observer (state estimator)
global A B C H
% constant parameters 
a_11 = 2;
a_12 = 1;
a_21 = 2;
a_22 = 1;

% epsilon 
epsilon = [0.002 0.0001];
e = epsilon(2);

A = [0 1; 0 0];
B = [0 1]';
C = [1 0];
H1= [a_11/e a_12/e^2]';
H2= [a_21/e a_22/e^2]';

A = blkdiag(A,A);
B = blkdiag(B,B);
C = blkdiag(C,C);
H = blkdiag(H1,H2);

%% Simulation
t_f = 25; % final simulation time
[t_sfb,x_sfb] = ode45(@x_dot,[0 t_f],[x0; 0]');
[t_ofb,x_ofb] = ode15s(@dx_cart_pole,[0 t_f],[x0; zeros(4,1); 0]');
N = numel(t_ofb);


% results from state feedback (sfb)
x_sfb_1 = wrapToPi(x_sfb(:,1)); % Angular displacement: [rad]
x_sfb_2 = x_sfb(:,2);           % Angular velocity:     [rad/s]
x_sfb_3 = x_sfb(:,3);           % Displacement:         [m]
x_sfb_4 = x_sfb(:,4);           % Velocity:             [m/s]
u_sfb   = x_sfb(:,5);           % Control input

% results from output feedback (ofb)
x_ofb_1 = wrapToPi(x_ofb(:,1));
x_ofb_2 = x_ofb(:,2);
x_ofb_3 = x_ofb(:,3);
x_ofb_4 = x_ofb(:,4);
u_ofb   = x_ofb(:,9);

%% Plots
figure(1)
hold on;
subplot(221)
plot(t_sfb,rad2deg(x_sfb_1),'-',t_ofb,rad2deg(x_ofb_1),'-.','linewidth',2),xlabel('Time [s]'),ylabel('$x_1$ [deg]','interpreter','latex'),grid
hold on;
subplot(222)
plot(t_sfb,rad2deg(x_sfb_2),'-','DisplayName','SFB','linewidth',2)
hold on
plot(t_ofb,rad2deg(x_ofb_2),'-.','DisplayName',['OFB \epsilon=',num2str(e)],'linewidth',2),xlabel('Time [s]'),ylabel('$x_2$ [deg/s]','interpreter','latex'),grid
legend('show')
hold on;
subplot(223)
plot(t_sfb,x_sfb_3,'-',t_ofb,x_ofb_3,'-.','linewidth',2),xlabel('Time [s]'),ylabel('$x_3$ [m]','interpreter','latex'),grid
hold on;
subplot(224)
plot(t_sfb,x_sfb_4,'-',t_ofb,x_ofb_4,'-.','linewidth',2),xlabel('Time [s]'),ylabel('$x_4$ [m/s]','interpreter','latex'),grid

figure(2)
hold on;
subplot(211)
plot(rad2deg(x_sfb_1),rad2deg(x_sfb_2),'-','DisplayName','SFB','linewidth',2),
hold on;
plot(rad2deg(x_ofb_1),rad2deg(x_ofb_2),'-.','DisplayName',['OFB \epsilon=',num2str(e)],'linewidth',2),xlabel('$\theta$', 'interpreter','latex'),ylabel('$\dot{\theta}$', 'interpreter','latex'),title('Phase Portraits'),grid
legend('show')
hold on;
subplot(212)
plot(x_sfb_3,x_sfb_4,'-',x_ofb_3,x_ofb_4,'-.','linewidth',2),xlabel('$x$', 'interpreter','latex'),ylabel('$\dot{x}$', 'interpreter','latex'),grid

figure(3)
hold on;
plot(t_sfb,u_sfb,'-','DisplayName','SFB','linewidth',2)
hold on;
plot(t_ofb,u_ofb,'-.','DisplayName',['OFB \epsilon=',num2str(e)],'linewidth',2),xlabel('Time [s]'),ylabel('$u$','interpreter','latex'),title('Control input'),grid
legend('show')

%% Animation
figure(4)
O = [x_ofb_3 zeros(length(x_ofb_3),1)]; % Origin
axis(gca); % Aspect ratio of the plot
axis([-25 25 -25 25]) % The limits of the plot
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.05, 0.05, 0.5, 0.8]);
xlabel('[m]')
ylabel('[m]')
title('Cart-Pole Simulation')
grid on;
pause('on')
% Loop for animation
for i=1:N
    % Mass point
    p = 2*L*[sin(x_ofb_1(i)) cos(x_ofb_1(i))];
    
    % Pendulum 
    pend = line([O(i,1) O(i,1)+p(1)],[O(i,2) O(i,2)+p(2)],'linewidth',3,'Color','k');
    % Circle in origin
    O_circ = viscircles(O(i,:),0.08,'Color','k');
    % Ball
    ball = viscircles(O(i,:)+p/2,0.01);
    % Cart
    width  = 2;
    height = 1;
    xLeft   = O(i,1) - width/2;
    yBottom = O(i,2) - height/2;
    cart = rectangle('Position',[xLeft yBottom width height]);
    left_wheel = viscircles(O(i,:)-[0.7,0.8],0.2,'Color','k');
    right_wheel = viscircles(O(i,:)+[0.7,-0.8],0.2,'Color','k');

    % Dynamic text
    t_label = sprintf('t = %.1f s', t_ofb(i));
    theta = sprintf('%c',952);
    x1 = sprintf('%.1f deg',rad2deg(x_ofb_1(i)));
    x1 = sprintf('%s = %s',theta,x1);
    x2 = sprintf('x = %.1f m',x_ofb_3(i));
    h1=text(4,9,t_label,'FontSize',14);
    h2=text(4,7,x1,'FontSize',14);
    h3=text(4,5,x2,'FontSize',14);
    drawnow();
        
    % Time interval to update the plot
    pause(1e-3);
    
    % Delete previous objects if it not the final loop
    if i<N
        delete(pend);
        delete(ball);
        delete(cart);
        delete(left_wheel);
        delete(right_wheel);
        delete(O_circ);
        delete(h1);
        delete(h2);
        delete(h3);
    end
end
%% Functions
function s=sat(u,sat_level)
    % sat is the saturation function with unit limits and unit slope.
    if abs(u)>sat_level
        s=sign(u)*sat_level;
    else 
        s=u;
    end
end
function phi = phi_nom(x,u)
global M m b L I g
%x(2) = sat(x(2),deg2rad(20));
%x(4) = sat(x(4),15);
phi = [(1/( (I+m*L^2)*(m+M) - m^2*L^2*cos(x(1))^2 )) * m*L * ( (m+M)*g*sin(x(1)) - cos(x(1))*(u + m*L*x(2)^2*sin(x(1)) - b*x(4)) );
       (1/( (I+m*L^2)*(m+M) - m^2*L^2*cos(x(1))^2 )) * ( -(m*L)^2*g*sin(x(1))*cos(x(1)) + (I+m*L^2)*(u + m*L*x(2)^2*sin(x(1)) - b*x(4)) )];
end
function u=u_c(x)
    global M m b L g kp ki kl
    % Partial state feedback control law
    eps = x(3) + kp/L*sin(x(1));
    eps_dot = x(4) + kp/L*x(2)*cos(x(1)); 
    eta = ki*eps + kp/L*(g/L*cos(x(1)) - x(2)^2)*sin(x(1)); 
    delta = 1 + kl - kp/L^2*cos(x(1))^2;
    v = -(eps_dot + eta)/delta;
    u = m*g*sin(x(1))*cos(x(1)) - m*L*x(2)^2*sin(x(1)) + b*x(4) + (M+m*sin(x(1))^2)*v;
end
function dx_total = x_dot(t,x)
    % x(1:4) : actual state
    global A B
    % partial state feedback control
    u = sat(u_c(x),200);
    % calculate derivatives
    dx = A*x(1:4) + B*phi_nom(x(1:4),u);
    du = u - x(5);
    dx_total = [dx; du];
end
function dx_total = dx_cart_pole(t,x)
    % x(1:4) : actual state
    % x(5:8) : estimated/observed state
    % x(9)   : control input
    global A B C H
    % measurement
    y = [x(1); x(3)];
    % partial output feedback control
    u = sat(u_c(x(5:8)),200); 
    % calculate derivatives
    dx = A*x(1:4) + B*phi_nom(x(1:4),u);
    dx_hat = A*x(5:8) + H*(y - C*x(5:8));
    %dx_hat = A*x(5:8) + B*phi_nom(x(5:8),u) + H*(y - C*x(5:8)); 
    du = u - x(9); % this is added so that we can plot u
    % concatenate result
    dx_total = [dx; dx_hat; du];
end
