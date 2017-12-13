clear all; close all;

method = "PRM";
path = strcat("path_", method, ".dat");
path0 = strcat("path0_", method, ".dat");
edge = strcat("edges_", method, ".dat");
vertices = strcat("vertices_", method, ".dat");
data_2d = load(path);
path0 = load(path0);
edge = load(edge);
vertices = load(vertices);
figure; hold on;
stanfordcolor = 1/255*[140,21,21];
plot(edge(:,1), edge(:,2), 'g-');
plot(vertices(:,1), vertices(:,2), 'bo');
plot(data_2d(:,1), data_2d(:,2), 'o-', 'linewidth', 1);
plot(path0(:,1), path0(:,2), 'x-', 'linewidth', 1);

rectangle('Position', [-0.5,-0.5,1,1], 'FaceColor', stanfordcolor);
legend('edge', 'smoothed path', 'raw path')
xlabel('x');ylabel('y');title(method);
axis equal;

method = "RRT";
path = strcat("path_", method, ".dat");
path0 = strcat("path0_", method, ".dat");
edge = strcat("edges_", method, ".dat");
vertices = strcat("vertices_", method, ".dat");
data_2d = load(path);
path0 = load(path0);
edge = load(edge);
vertices = load(vertices);
figure; hold on;
stanfordcolor = 1/255*[140,21,21];
plot(edge(:,1), edge(:,2), 'g-');
plot(vertices(:,1), vertices(:,2), 'bo');
plot(data_2d(:,1), data_2d(:,2), 'o-', 'linewidth', 1);
plot(path0(:,1), path0(:,2), 'x-', 'linewidth', 1);

rectangle('Position', [-0.5,-0.5,1,1], 'FaceColor', stanfordcolor);
legend('edge', 'smoothed path', 'raw path')
xlabel('x');ylabel('y');title(method);
axis equal;