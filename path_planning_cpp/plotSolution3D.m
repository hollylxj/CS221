close all; clear all;

method = "RRT";
specific = load(strcat("simple3_", method, ".txt"));
uniform = load(strcat("simple1_", method, ".txt"));
spec_raw = load(strcat("diy_", method, ".txt"));
uni_raw = load(strcat("uniform_", method, ".txt"));

figure; hold on;
plot3([0;0], [0,0], [0,1], 'o', 'linewidth', 2)
uni = plot3(uniform(:,1), uniform(:,2), uniform(:,3), 'bo-');
spec = plot3(specific(:,1), specific(:,2), specific(:,3), 'co-');
specr = plot3(spec_raw(:,1), spec_raw(:,2), spec_raw(:,3), 'ko-');
unir = plot3(uni_raw(:,1), uni_raw(:,2), uni_raw(:,3), 'mo-');
xlabel('x');ylabel('y');zlabel('z');

axis([-1 1 -1 1 -0.3 1]), grid on, rotate3d on;
X = [1;0.8;0.8;1;1];
Y = [1;1;0.8;0.8;1];
Z = [0.25;0.25;0.25;0.25;0.25];
plotsqr(X,Y,Z); plotsqr(-X,Y,Z); plotsqr(X,-Y,Z); plotsqr(-X,-Y,Z)
legend([uni, spec, unir, specr], {'uni. sampler','spec. sampler', 'uni. raw',...
    'spec. raw'})
title(strcat(method, ' 3D Path Planning'))


method = "PRM";
specific = load(strcat("simple3_", method, ".txt"));
uniform = load(strcat("simple1_", method, ".txt"));
spec_raw = load(strcat("diy_", method, ".txt"));
uni_raw = load(strcat("uniform_", method, ".txt"));

figure; hold on;
plot3([0;0], [0,0], [0,1], 'o', 'linewidth', 2)
uni = plot3(uniform(:,1), uniform(:,2), uniform(:,3), 'bo-');
spec = plot3(specific(:,1), specific(:,2), specific(:,3), 'co-');
specr = plot3(spec_raw(:,1), spec_raw(:,2), spec_raw(:,3), 'ko-');
unir = plot3(uni_raw(:,1), uni_raw(:,2), uni_raw(:,3), 'mo-');
xlabel('x');ylabel('y');zlabel('z');

axis([-1 1 -1 1 -0.3 1]), grid on, rotate3d on;
X = [1;0.8;0.8;1;1];
Y = [1;1;0.8;0.8;1];
Z = [0.25;0.25;0.25;0.25;0.25];
plotsqr(X,Y,Z); plotsqr(-X,Y,Z); plotsqr(X,-Y,Z); plotsqr(-X,-Y,Z)
legend([uni, spec, unir, specr], {'uni. sampler','spec. sampler', 'uni. raw',...
    'spec. raw'})
title(strcat(method, ' 3D Path Planning'))

function plotsqr(X,Y,Z)
    plot3(X, Y, Z, 'r');
    plot3(X, Y, Z+0.25, 'r');
    for k=1:length(X)-1
        plot3([X(k);X(k)],[Y(k);Y(k)],[0.25;0.5], 'r');
    end
end
