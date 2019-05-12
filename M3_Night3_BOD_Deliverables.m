% Spinning thing bridge of doom

a = 0.4
l = 0.4 

syms u;
alpha = 1;
r = [(-2*a*((l-cos(u))*cos(u)+(1-l))); (2*a*(l-cos(u))*sin(u)); 0];
That = diff(r)/norm(diff(r));
Nhat = diff(That)/norm(diff(That))

u = alpha*linspace(0,3.2,50);
That = double(subs(That, u)); %substitute u values into function
r = double(subs(r, u)); %substitute u values into function
Nhat = double(subs(Nhat, u)); %substitute u values into function

for n = 1:length(u)% loop through each of the points
    plot(r(1,:),r(2,:)), axis([-2 2 -2 2]); hold on % plot the entire curve
%     axis auto;
    xlabel('x');
    ylabel('y');
    quiver(r(1,n),r(2,n),That(1,n),That(2,n),'r') % plot the unit tangent
    quiver(r(1,n),r(2,n),Nhat(1,n),Nhat(2,n),'b') % plot the unit normal
    legend('That', 'Nhat')

    drawnow % force the graphic to update as it goes
end
hold off % plot the unit binormal

%% 2

%Deliverable 2

clear;

a = 0.4
l = 0.4 

syms t;
syms u;
alpha = .224;

r = [(-2*a*((l-cos(alpha*t))*cos(alpha*t)+(1-l))); (2*a*(l-cos(alpha*t))*sin(alpha*t)); 0];
That = diff(r)/norm(diff(r));
Nhat = diff(That)/norm(diff(That))

u = linspace(0,3.2,50);
t = linspace(0, pi/alpha, 50);
w = cross(That, diff(That));

That = double(subs(That, t)); %substitute u values into function
r = double(subs(r, t)); %substitute u values into function
Nhat = double(subs(Nhat, t)); %substitute u values into function

w = double(subs(w, t));
d = 0.255;
 
velocity = That;
linspeed = vecnorm(velocity);
vleft = linspeed + w(3,:) * (d/2);
vright = linspeed - w(3,:) * (d/2);

plot(t, vleft);
hold on;
plot(t, vright);
legend('vleft', 'vright')
xlabel('time')
ylabel('velocity')
title('Respective wheel velocities')
hold off;

%Deliverable 3

figure;
plot(t, linspeed)
axis([0 15 -1 2]);
hold on;
plot(t, w)
xlabel('time')
ylabel('velocity')
title('Velocity and Acceleration')
legend('linear speed', 'angular velocity')
hold off;
%% Deliverable 4 calculations
clear;

a = 0.4
l = 0.4 

syms t;
syms u;
% alpha = .224;
alpha = .22

%do functions symbolically first
r = [(-2*a*((l-cos(alpha*t))*cos(alpha*t)+(1-l))); (2*a*(l-cos(alpha*t))*sin(alpha*t)); 0];
That = diff(r)/norm(diff(r));
Nhat = diff(That)/norm(diff(That))

% initialize variables essential to actual function values
u = linspace(0,3.2,50);
t = 0:0.1:pi/alpha;
w = cross(That, diff(That));

% subtitute essential variable into functions
That = double(subs(That, t)); %substitute u values into function
% r = double(subs(r, t)); %substitute u values into function
Nhat = double(subs(Nhat, t)); %substitute u values into function
w = double(subs(w, t));
d = 0.255;
 
velocity = diff(r);
velocity = double(subs(velocity, t));

linspeed = vecnorm(velocity);
vleft = linspeed + w(3,:) * (d/2);
vright = linspeed - w(3,:) * (d/2);

%% Deliverable 4, Running the robot (Depends on above section)

pub = rospublisher('/raw_vel');
msg = rosmessage(pub);

% get the robot moving
msg.Data = [0.0, 0.0];
send(pub, msg);

for i = 1:numel(t) % iterate through indices in timestep array
    msg.Data = [vright(:, i), vleft(:, i)];
    send(pub, msg); %DOASFMOSAPNFODPSAJFOI DSAIA IT FUCKING WORKED
    pause(0.1);
end
msg.Data = [0,0];
send(pub, msg);

%% Deliverable 5

%that's not fair, it is also far, i am also slow, 
% all hail JeffEmMarJohn TowGeddeSomerviDusek

%% Deliverable 6 (depends on deliverable 4)

%plot wheel velocities from dataset on deliverable 4

load('Deliverable6.mat')
close all;

a = 0.4
l = 0.4 
syms t;
syms u;
alpha = .224;
d = .255;

r = [(-2*a*((l-cos(alpha*t))*cos(alpha*t)+(1-l))); (2*a*(l-cos(alpha*t))*sin(alpha*t)); 0];
That = diff(r)/norm(diff(r));
velocity = That;
Nhat = diff(That)/norm(diff(That))
t = 0:0.105:pi/alpha;


That = double(subs(That, t)); %substitute u values into function
r = double(subs(r, t)); %substitute u values into function
Nhat = double(subs(Nhat, t)); %substitute u values into function
velocity = That;

time = dataset(:, 1);
velocityleft = [diff(dataset(:, 2))./diff(time); 0];
velocityright = [diff(dataset(:, 3))./diff(time); 0];

figure('Name', 'wheels');
plot(dataset(:, 1), velocityleft);
hold on;
plot(dataset(:, 1), velocityright);
% axis([0 25 1 4])
xlabel('Time')
ylabel('Velocity')
legend('Left Wheel Velocity', 'Right Wheel Velocity')
title('Wheel Velocities')
hold off;

linspeed = (velocityleft + velocityright)./2;
% linvel = linspe

% w_vel = cross(linvel, (vecnorm(linvel)));
w_vel = (-velocityleft + velocityright)./d;

figure('Name', 'w_vel');
plot(w_vel), hold on;
plot(linspeed)
xlabel('time')
legend('angular velocity', 'linear velocity')
hold off;

theta = cumsum(w_vel .* [diff(time); 0]) - pi/2; %period operator (.* ./) is elemental operation
vx = linspeed.*cos(theta); %this returns an arc correspondent x component of angular velocity
vy = linspeed.*sin(theta); %this returns an arc correspondent y component of angular velocity

figure;
xpos = cumsum(vx.*[diff(time); 0]); %the xpos was acquired by multiplying the angular velocity by the time elapsed and the current angular velocity
ypos = cumsum(vy.*[diff(time); 0]);
plot(xpos, ypos), axis image
xlabel('x-position')
ylabel('y-position')
title('coordinate positions')
hold off;

figure('Name', '(x, y) position');
plot(vx), hold on; 
plot(vy)
legend('xpos', 'ypos')
title('linear and angular positions')
xlabel('time')
ylabel('position')
hold off;

figure('Name', 'Experimental vs Theoretical')
quiver(xpos(1:4:end),ypos(1:4:end), cos(theta(1:4:end)),sin(theta(1:4:end)), '-.>c'), hold on;
quiver(r(1, 1:4:end), r(2, 1:4:end), velocity(1, 1:4:end), velocity(2, 1:4:end), 'dr')
legend('experimental', 'theoretical')
axis image;
hold off;

%Calculating error I hope

%% Error
close all;
coord_diff = (xpos-r(1,:)).^2 + (ypos-r(2, :)).^2
err_pos = sqrt(coord_diff)./length(coord_diff)
figure;
title('Error')
plot(t, err_pos(1,:))
hold on;
plot(t, err_pos(2,:))
hold off;

%% RMSE
close all;
coords = mean(xpos-r(1,:)).^2; mean(ypos-r(2,:)).^2
coords_sqrt = sqrt(coords)./length(coords)
plot(t, coords_sqrt(1,:)), hold on;
plot(t, coords_sqrt(2,:)), hold off;
