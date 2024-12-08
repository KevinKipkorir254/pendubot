 clc; 
 m1 = 0.48763073080899393963;  %% Mass of cart (kg)
 m2 = 0.48763073080899393963;  %% Mass of pendulum (kg)
 l1 = 0.4;  %% Length of pendulum (m)
 l2 = 0.4;
 lc1 = l1/2;
 lc2 = l2/2;
 I1 = 0.0025009745471827196997;
 I2 = 0.0025009745471827196997;
 g = 9.81; %% Gravitational acceleration (m/s^2)

 theta_1 = m1 * lc1 * lc1 + m2 * l1 * l1 + I1;
 theta_2 = m2 * lc2 * lc2 + I1;
 theta_3 = m2 * l1 * lc2;
 theta_4 = m1 * lc1 + m2 * l1;
 theta_5 = m2 * lc2;

 %% q1
 %% q1_dot
 %% q2
 %% q2_dot

 d21 = (theta_2*theta_4 - theta_3*theta_5)*g/(theta_1*theta_2 - theta_3*theta_3);
 d23 = -1*(theta_3*theta_5*g)/(theta_1*theta_2 - theta_3*theta_3);

 d41 = (theta_5*g*(theta_1 + theta_3) - theta_4*g*(theta_2 + theta_3))/(theta_1*theta_2 - theta_3*theta_3);
 d43 = (theta_5*g*(theta_1 + theta_3))/(theta_1*theta_2 - theta_3*theta_3);

 b2 = (theta_2)/(theta_1*theta_2 - theta_3*theta_3);
 b4 = (-theta_2 - theta_3)/(theta_1*theta_2 - theta_3*theta_3);

 A = [ 0    1   0   0;
       d21  0  d23  0;
       0    0   0   1;
       d41  0  d43  0;
      ];

 B = [ 0;
       b2;
       0;
       b4;
      ];



Q = [ 2       0     0     0;
      0       2     0     0;
      0       0     2000    0;
      0       0      0    2;
      ];

R = [1];

[K,P,E] = lqr(A,B,Q,R);

fprintf("%f, %f, %f, %f\n", K(1), K(2), K(3), K(4))

AA = A - B*K; % Closed-loop dynamics
BB = B;
CC = eye(4);  % Output all states
DD = zeros(4, 1); % No direct input to output

% Simulate with initial conditions
x0 = [0.1; 0; -0.1; 0]; % Initial conditions: small perturbations
t = 0:0.01:10;  % Simulation time

% Simulate the system using initial conditions
[y, t, x] = initial(ss(AA, B, CC, DD), x0, t);

% Plot the results
figure;
plot(t, x(:,1), t, x(:,2), t, x(:,3), t, x(:,4));
grid on;
title('Response Curves for x, x_d, \theta, \theta_d versus t');
xlabel('Time (sec)');
ylabel('State Variables');
legend('x (Cart Position)', 'x_d (Cart Velocity)', '\theta (Pendulum Angle)', '\theta_d (Pendulum Angular Velocity)');
