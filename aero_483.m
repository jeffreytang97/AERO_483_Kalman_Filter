% Aero 483 Project
% By: Jeffrey Tang
% Submission date: April 17th, 2020

% The headers to explain the functions are below.

% Read input from ins and gps text file provided
ins_matrix = dlmread('ins.txt');
gps_matrix = dlmread('gps.txt');
disp('ins data:');
disp(ins_matrix);
disp('gps data:');
disp(gps_matrix);

% Initialize variable for state estimate equations in discrete time and other variables
x_next = 0; y_next = 0;                             % Position components
Vx_next = 0; Vy_next = 0;                           % Velocity components          
g = 9.8;                                            % gravity of earth in m/s^2
x_next_n = 0; y_next_n = 0;                         % Position components with random noise
Vx_next_n = 0; Vy_next_n = 0;                       % Velocity components with random noise
number_of_rows_ins = 121;
number_of_rows_gps = 12;
T = 0.05;                                           % Time period update of 0.05 seconds 

% This is to generate the GPS array data for plotting purposes
for row = 1:number_of_rows_gps
    time_array_gps(row) = gps_matrix(row,1);
    x_data_array(row) = gps_matrix(row,2);   
    y_data_array(row) = gps_matrix(row,3);
end


% # ---------------------------------- Simulation of quadrotor without noise ---------------------------------- #

%{
Header 1:

The first part is the simulation of quadrotor without the random noises.
From calculation made on paper, I was able to determine the equations and
matrices needed for the discrete time state estimation. The function starts
with the update of the state vector of x, y, Vx, and Vy. At t=0, x_next,
y_next, Vx_next and Vy_next are all at 0. Then, we declare the Phi matrix,
Gamma matrix, state vector matrix and U matrix which is the control inputs.
Then, we use those matrices to calculate the following:
X_next_state = (phi_matrix * x_current_state) + (gamma_matrix * u_matrix).
This is calculate in a for loop in order to handle every data available.
Each state estimate is calculated once per iteration. Each iteration has a
time period of 0.05s and it ends at 6s.

This step is for question 1 and 2. It is to plot the pitch and roll angle
in function of time and also to compare the state estimate from the INS
data and GPS data.
%}

for row = 1:number_of_rows_ins               
    
    % The ins angle data are in radians. We must convert them to degrees.
    theta_pitch = radtodeg(ins_matrix(row,2));
    phi_roll = radtodeg(ins_matrix(row,3)); 
    
    % No noise update
    x = x_next;
    y = y_next;
    Vx = Vx_next;
    Vy = Vy_next;
    
    % For plotting purposes
    x_ins(row) = x;
    y_ins(row) = y;
    
    % Creating matrices for time, pitch, roll, position x and y for
    % plotting purposes for question 1. 
    time_array_ins(row) = ins_matrix(row,1);
    theta_pitch_data_array(row) = theta_pitch;
    phi_roll_data_array(row) = phi_roll; 
    
    % Initialize matrices per iteration for the state estimate
    % equation. (PHI, GAMMA, x_state_vector and u matrix)
    phi_matrix = [1 0 T 0;
                  0 1 0 T;
                  0 0 1 0;
                  0 0 0 1];
              
    v = 0.5*(T^2);
    gamma_matrix = [T 0 v 0;
                    0 T 0 v;
                    0 0 T 0;
                    0 0 0 T];
                
    x_current_state = [x; y; Vx; Vy];
    u_matrix = [0; 0; g*tand(theta_pitch); g*tand(phi_roll)];
    
    % Solve for X_dot_matrix without random noise
    % Equation: xk+1 = phi*xk + Gamma*u
    X_next_state = (phi_matrix * x_current_state) + (gamma_matrix * u_matrix);
    
    % Pass the next state values
    x_next = X_next_state(1);
    y_next = X_next_state(2);
    Vx_next = X_next_state(3);
    Vy_next = X_next_state(4);
end

% Plot for pitch and roll angle
figure(1)
plot(time_array_ins, theta_pitch_data_array,'b', time_array_ins, phi_roll_data_array, 'r');
title('Pitch angle(theta) and Roll angle(phi) in function time (t)')
xlabel('t (seconds)');
ylabel('Pitch and Roll (Degrees)');
legend('Pitch Angle', 'Roll Angle', 'Location', 'SouthEast');

%{
Header 2:
We plot the trajectory in a loop first to show it in real time.
%}
for i=1:121
    figure(2)
    plot(x_ins(i), y_ins(i),'or','MarkerSize',5,'MarkerFaceColor','b');
    axis([-2 7 -2 7]);
    title('Trajectory Plot live update (0.05s)');
    xlabel('X (m)');
    ylabel('Y (m)');
    hold on;
    pause(0.05);
end

figure(3)
plot(x_ins, y_ins, 'b', x_data_array, y_data_array, 'r');
xlabel('X (m)');
ylabel('Y (m)');
title('Simulation/Trajectory Plot without Random Noise');
legend('Trajectory from INS data', 'Trajectory from GPS data', 'Location', 'SouthEast');

figure(4)
plot(x_data_array, y_data_array, 'r');
xlabel('X (m)');
ylabel('Y (m)');
title('GPS Points Trajectory');
legend('Trajectory from GPS data', 'Location', 'SouthEast');


% # ----------------------------------- Simulation of quadrotor with random noise ---------------------------------- #    
%{
Header 3:

This part is the same as previously for question 2, except that we must
include the random noises and we must run it 10x to compare the
trajectories.
At the end, we plot the 10 INS state estimate trajectories and compare
them.
%}

% Run it 10 times
for i = 1:10
    % Re-initialize these arrays and data
    x_ins_n = [];
    y_ins_n = [];
    x_next_n = 0;
    y_next_n = 0;
    Vx_next_n = 0;
    Vy_next_n = 0;
    
    for row = 1:number_of_rows_ins                              
        theta_pitch_n = radtodeg(ins_matrix(row,2));
        phi_roll_n = radtodeg(ins_matrix(row,3)); 

        % With noise update
        x_n = x_next_n;
        y_n = y_next_n;
        Vx_n = Vx_next_n;
        Vy_n = Vy_next_n;

        % Information for the trajectory of drone plot
        x_ins_n(row) = x_n;
        y_ins_n(row) = y_n;

        % Initialize matrices per iteration for the state estimate
        % equation. (PHI, GAMMA, x_state_vector and u matrix)
        phi_matrix_n = [  1 0 T 0;
                          0 1 0 T;
                          0 0 1 0;
                          0 0 0 1];

        h = 0.5*(T^2);
        gamma_matrix_n = [  T 0 h 0;
                            0 T 0 h;
                            0 0 T 0;
                            0 0 0 T];

        x_current_state_n = [x_n; y_n; Vx_n; Vy_n];
        u_matrix_n = [0; 0; g*tand(theta_pitch_n); g*tand(phi_roll_n)];

        % Solve for X_dot_matrix with random noises Wv,x and Wv,y
        % Equation: xk+1 = phi*xk + Gamma*u + W
        Wv_x = normrnd(0,0.03);
        Wv_y = normrnd(0,0.03);
        w_random_noise = [0; 0; Wv_x; Wv_y];   % Doing the covariance of the random noises matrix in order to convert into discrete time.
        X_next_state_n = (phi_matrix_n * x_current_state_n) + (gamma_matrix_n * u_matrix_n) + w_random_noise;
        
        % Pass the next state values
        x_next_n = X_next_state_n(1);
        y_next_n = X_next_state_n(2);
        Vx_next_n = X_next_state_n(3);
        Vy_next_n = X_next_state_n(4);
    end
    
    % This is to store each of the arrays separately of the 10 trajectories
    % for plotting purposes.
    if i==1
        x_ins_n1 = x_ins_n;
        y_ins_n1 = y_ins_n;
    elseif i==2
        x_ins_n2 = x_ins_n;
        y_ins_n2 = y_ins_n;
    elseif i==3
        x_ins_n3 = x_ins_n;
        y_ins_n3 = y_ins_n;
    elseif i==4
        x_ins_n4 = x_ins_n;
        y_ins_n4 = y_ins_n;
    elseif i==5
        x_ins_n5 = x_ins_n;
        y_ins_n5 = y_ins_n;
    elseif i==6
        x_ins_n6 = x_ins_n;
        y_ins_n6 = y_ins_n;
    elseif i==7
        x_ins_n7 = x_ins_n;
        y_ins_n7 = y_ins_n;
    elseif i==8
        x_ins_n8 = x_ins_n;
        y_ins_n8 = y_ins_n;
    elseif i==9
        x_ins_n9 = x_ins_n;
        y_ins_n9 = y_ins_n;
    else
        x_ins_n10 = x_ins_n;
        y_ins_n10 = y_ins_n;
    end
end

figure(5)
plot(x_ins_n8, y_ins_n8, 'r');
xlabel('X (m)');
ylabel('Y (m)');
title('Trajectory Plot With Random Noises (10x)');
hold on;
plot(x_ins_n2, y_ins_n2, 'm');
hold on;
plot(x_ins_n3, y_ins_n3, 'c'); 
hold on;
plot(x_ins_n4, y_ins_n4, 'g'); 
hold on;
plot(x_ins_n5, y_ins_n5, 'b'); 
hold on;
plot(x_ins_n6, y_ins_n6, 'k'); 
hold on;
plot(x_ins_n7, y_ins_n7, 'y'); 
hold on;
plot(x_ins_n8, y_ins_n8, 'r'); 
hold on;
plot(x_ins_n9, y_ins_n9, 'b'); 
hold on;
plot(x_ins_n10, y_ins_n10, 'm'); 
legend('trajectory 1', 'trajectory 2', 'trajectory 3', 'trajectory 4', 'trajectory 5', 'trajectory 6', 'trajectory 7', 'trajectory 8', 'trajectory 9', 'trajectory 10', 'Location', 'NorthWest');


% # ---------------------------------- Kalman Filter design and state estimate ------------------------------------ #    

%{
Header 4:

Finally, this part is the design of the Kalman filter. We use the Kalman
filter to compare it to the state estimate of INS and GPS data.

The Kalman filter is also run in a loop. The function start to calculate
all the components needed for x_k_plus_next and P_k_plus_next. The result
will then be passed to the next loop to calculate the next state estimate.
At each iteration, the coordinates of x and y calculated by the kalman
filter will be stored in two different arrays (x_kalman and y_kalman) and
used for plotting purposes.

We use the equations provided from the course material to calculate the
state estimate at each iteration.

%}

P_k_plus_next = [0 0 0 0;
            0 0 0 0;
            0 0 0 0;
            0 0 0 0];
R = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
 
gps_variance = 0.3;
R_k = (0.3)^2 * R;

x_next_k = 0;
y_next_k = 0;
Vx_next_k = 0;
Vy_next_k = 0;

for row = 1:number_of_rows_ins                                  
        theta_pitch_k = radtodeg(ins_matrix(row,2));
        phi_roll_k = radtodeg(ins_matrix(row,3)); 

        % With noise update
        x_k = x_next_k;
        y_k = y_next_k;
        Vx_k = Vx_next_k;
        Vy_k = Vy_next_k;
        P_k_plus_current = P_k_plus_next;

        % Information for the trajectory of drone plot
        x_kalman(row) = x_k;
        y_kalman(row) = y_k;

        % Initialize matrices per iteration for the state estimate
        % equation. (PHI, GAMMA, x_state_vector and u matrix)
        phi_matrix_k = [  1 0 T 0;
                          0 1 0 T;
                          0 0 1 0;
                          0 0 0 1];

        h = 0.5*(T^2);
        gamma_matrix_k = [  T 0 h 0;
                            0 T 0 h;
                            0 0 T 0;
                            0 0 0 T];

        x_k_plus_current = [x_k; y_k; Vx_k; Vy_k];
        u_matrix_k = [0; 0; g*tand(theta_pitch_k); g*tand(phi_roll_k)];
        
        H_matrix = [1 0 0 0;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];

        % Solve for x_k
        Wv_x = normrnd(0,0.03);
        Wv_y = normrnd(0,0.03);
        Q_matrix = cov([0; 0; Wv_x; Wv_y]);
        x_k = (phi_matrix_k * x_k_plus_current) + (gamma_matrix_k * u_matrix_k) + Q_matrix;
        y_k = H_matrix * x_k;
        
        % Solve for x_k_minus
        x_k_minus = (phi_matrix_k * x_k_plus_current) + (gamma_matrix_k * u_matrix_k);
        y_k_minus = H_matrix * x_k_minus;
        
        % Solve for Kalman gain
        P_k_minus = (phi_matrix_k * P_k_plus_current * transpose(phi_matrix_k)) + Q_matrix;
        Kalman_gain = transpose((H_matrix * P_k_minus)) * inv((H_matrix * P_k_minus * transpose(H_matrix)) + R_k);
        
        % Solve for state estimate x_k_plus_next
        x_k_plus_next = x_k_minus + (Kalman_gain * (y_k - y_k_minus));
        
        % Pass the next state values
        x_next_k = x_k_plus_next(1);
        y_next_k = x_k_plus_next(2);
        Vx_next_k = x_k_plus_next(3);
        Vy_next_k = x_k_plus_next(4);
        
        % Solve for P_k_plus_next
        I = eye(4);
        P_k_plus_next = (I - (Kalman_gain * H_matrix)) * P_k_minus;
        disp(P_k_plus_next);
end

% This plot is to compare the trajectories from GPS data, INS and Kalman filter
% state estimates.

figure(6)
plot(x_ins, y_ins, 'b', x_data_array, y_data_array, 'r', x_kalman, y_kalman, 'g');
xlabel('X (m)');
ylabel('Y (m)');
title('Simulation/Trajectory Plot Comparison between INS, GPS and Kalman Filter');
legend('Trajectory from INS data', 'Trajectory from GPS data', 'Trajectory from Kalman Filter estimate', 'Location', 'SouthEast');
