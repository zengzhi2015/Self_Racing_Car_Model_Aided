close all
clear
clc

%% Test the calculation of theta
num_rows = 3;
num_cols = 4;
for r = 1:num_rows
    for j = 1:num_cols
        subplot(num_rows,num_cols,(r-1)*num_cols+j)
        width = 1.2+(rand-0.5)*2*0.1; % the width og the runway
        delta = 0.4+(rand-0.5)*2*0.1; % the width of the openning
        p1 = sign(rand-0.5); % the direction of the front openning (left:-1;right:1)
        p2 = sign(rand-0.5); % the direction of the back openning (left:-1;right:1)
        len = 1.5+(rand-0.5)*2*0.5; % the length of the runway block
        alpha = (rand-0.5)*2*pi/3; % the direction of the runway
        Cx = (rand-0.5)*2*width/2; % the shift of the runway in x
        Cy = (rand-0.5)*2*len/2; % the shift of the runway in y
        directions = pi:-pi/180:0; % the directions of LIDAR rays
        rmin = 0.2; % the minimum turning radius of the car.
        args = f_para2rect( width, delta, p1, p2, len, alpha, Cx, Cy );
        distances = f_blockdet( args, directions );
        distances(distances>2) = 2;
        %Calculation
        theta = f_caldirection_mod_2( p1,len,width,Cx,Cy,delta,rmin );
        theta = theta+alpha;
        theta = min(pi/9,max(-pi/9,theta));
        % Plot
        figure(1)
        f_plotcar()
        f_plotblock( args )
        f_plotrays( directions,distances )
        f_plotdirection( theta, 2 )
        xlim([-1.5,1.5])
        ylim([-1,2])
%         xlabel('x')
%         ylabel('y')
        axis off
        box on
    end
end



%% Generate the dataset
num_samples = 4000;
directions = pi:-pi/180:0; % the directions of LIDAR rays
training = zeros(num_samples,182);
for i=1:num_samples
    width = 1.2+(rand-0.5)*2*0.1; % the width og the runway
    delta = 0.4+(rand-0.5)*2*0.1; % the width of the openning
    p1 = sign(rand-0.5); % the direction of the front openning (left:-1;right:1)
    p2 = sign(rand-0.5); % the direction of the back openning (left:-1;right:1)
    len = 1.5+(rand-0.5)*2*0.5; % the length of the runway block
    alpha = (rand-0.5)*2*pi/3; % the direction of the runway
    Cx = (rand-0.5)*2*width/2; % the shift of the runway in x
    Cy = (rand-0.5)*2*len/2; % the shift of the runway in y
    directions = pi:-pi/180:0; % the directions of LIDAR rays
    rmin = 0.2; % the minimum turning radius of the car.
    args = f_para2rect( width, delta, p1, p2, len, alpha, Cx, Cy );
    distances = f_blockdet( args, directions );
    distances(distances>2) = 2;
    %Calculation
    theta = f_caldirection_mod_2( p1,len,width,Cx,Cy,delta,rmin );
    theta = theta+alpha;
    theta = min(pi/9,max(-pi/9,theta));
    training(i,1) = -theta;
    training(i,2:182) = round(distances*1000);
end

%% scrutinize the result
figure(2)
num = 952;
f_plotrays( directions,training(num,2:182)/1000 )
f_plotdirection( -training(num,1), 2 )
axis equal
%% Extra fake data for no detection
extra = zeros(2000,182);
extra(:,2:182) = 2000;
%%
figure(3)
num = 283;
f_plotrays( directions,extra(num,2:182)/1000 )
f_plotdirection( -extra(num,1), 2 )
%% Further test
width = 1.2; % the width og the runway
delta = 0.4; % the width of the openning
p1 = 1; % the direction of the front openning (left:-1;right:1)
p2 = 1; % the direction of the back openning (left:-1;right:1)
len = 1; % the length of the runway block
alpha = 0; % the direction of the runway
rmin = 0.2; % the minimum turning radius of the car.
Cx = -width/2:0.05:width/2;% the shift of the runway in x
Cy = -len/2:0.05:width/2;% the shift of the runway in x
num_Cx = length(Cx);
num_Cy = length(Cy);
[X,Y] = meshgrid(Cx, Cy);
Z = zeros(num_Cy,num_Cx);
%%
for i = 1:num_Cy
    for j = 1:num_Cx
        theta = f_caldirection_mod_2( p1,len,width,Cx(j),Cy(i),delta,rmin );
        theta = theta+alpha;
        %theta = min(pi/9,max(-pi/9,theta));
        Z(i,j) = theta;
    end
end
%%
surf(X,Y,Z)
shading interp
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
%% qiver plot
DX = cos(Z-pi/2);
DY = sin(Z-pi/2);
quiver(-X,-Y,-DX,-DY)
xlabel('x')
ylabel('y')
axis equal
