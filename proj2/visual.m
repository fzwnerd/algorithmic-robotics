%% environment 1
figure;
axis equal
xlim([-5, 5]);
ylim([-5, 5]);
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-3 -2.5 1 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -1.25 0.5 3.75], 'FaceColor',[0 .5 .5]);

%% environment 2
figure;
axis equal
xlim([-5, 5]);
ylim([-5, 5]);
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-4.3 -2.5 3 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -2.75 1 1], 'FaceColor',[0 .5 .5]);

%% point robot, enviroment 1
% obstacles
figure;
axis equal
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-3 -2.5 1 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -1.25 0.5 3.75], 'FaceColor',[0 .5 .5]);
hold on
% robot

p = [
-2.75 -2.75 
3.37554 -4.23791 
3.13617 0.828861 
1.16946 1.90333 
4.57266 4.38647 
4.9 0 
];

for i = 1:size(p, 1)-1
    p1 = p(i, :);
    plot(p1(1), p1(2), 'r.');
    p2 = p(i+1,:);
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'Color', [1 0 0]);
end
plot(p(1,1), p(1,2), 'r*');
plot(p(end,1), p(end, 2), 'r*');
xlim([-5, 5]);
ylim([-5, 5]);
title('Environment 1, point robot');
hold off

%% point, environment 2
figure;
axis equal
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-4.3 -2.5 3 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -2.75 1 1], 'FaceColor',[0 .5 .5]);
hold on

p = [
-2.75 -2.75 
3.3939 -3.28035 
-4.59689 -3.30487 
1.55722 -3.37367 
4.95335 -2.99545 
4.9 0
];

for i = 1:size(p, 1)-1
    p1 = p(i, :);
    plot(p1(1), p1(2), 'r.');
    p2 = p(i+1,:);
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'Color', [1 0 0]);
end
plot(p(1,1), p(1,2), 'r*');
plot(p(end,1), p(end, 2), 'r*');
xlim([-5, 5]);
ylim([-5, 5]);
title('Environment 2, point robot');
hold off

%% square, environment 1
figure;
axis equal
xlim([-5, 5]);
ylim([-5, 5]);
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-3 -2.5 1 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -1.25 0.5 3.75], 'FaceColor',[0 .5 .5]);
hold on
p = [
   -3.5 -3.5 0 
-3.51594 -2.82519 -0.124404 
-3.53187 -2.15037 -0.248807 
-3.54781 -1.47556 -0.373211 
-3.56375 -0.800749 -0.497614 
-3.57968 -0.125937 -0.622018 
-3.59562 0.548876 -0.746421 
-3.61156 1.22369 -0.870825 
-3.62749 1.8985 -0.995229 
-3.64343 2.57331 -1.11963 
-3.65937 3.24813 -1.24404 
-3.6753 3.92294 -1.36844 
-3.69124 4.59775 -1.49284 
-2.99197 4.60627 -1.30299 
-2.2927 4.61479 -1.11314 
-1.59343 4.62331 -0.923283 
-0.894159 4.63183 -0.733429 
-0.194889 4.64036 -0.543576 
0.504381 4.64888 -0.353722 
1.20365 4.6574 -0.163869 
1.90292 4.66592 0.0259845 
2.60219 4.67444 0.215838 
3.30146 4.68296 0.405691 
4.00073 4.69148 0.595545 
4.7 4.7 0.785398 
];
ll = [-0.15;-0.15;1];
lr = [0.15;-0.15;1];
ul = [-0.15;0.15;1];
ur = [0.15;0.15;1];

for i = 1:size(p,1)
    x = p(i,1);
    y = p(i,2);
    theta = p(i,3);
    rtMat = [cos(theta), -sin(theta),x;
        sin(theta),cos(theta), y;
        0,0,1];
    newll = (rtMat*ll)';
    newlr = (rtMat*lr)';
    newul = (rtMat*ul)';
    newur = (rtMat*ur)';
    %[newll;newlr;newul;newur]    
    plot([newll(1), newlr(1)],[newll(2),newlr(2)], 'r', ...
        [newll(1), newul(1)], [newll(2),newul(2)], 'r', ...
        [newul(1), newur(1)],[newul(2),newur(2)], 'r', ...
        [newlr(1), newur(1)], [newlr(2),newur(2)], 'r');
end
title('Environment 1, square robot');
hold off   

%% square, environment 2
figure;
axis equal
xlim([-5, 5]);
ylim([-5, 5]);
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-4.3 -2.5 3 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -2.75 1 1], 'FaceColor',[0 .5 .5]);
hold on

p = [
-3.5 -3.5 0 
-2.69955 -3.83764 0.65149 
-1.89909 -4.17528 1.30298 
-1.06879 -3.72429 1.75547 
-0.238483 -3.27329 2.20797 
0.591821 -2.8223 2.66046 
1.42212 -2.37131 3.11296 
2.25243 -1.92032 -2.71774 
3.08273 -1.46933 -2.26524 
2.9903 -0.461624 -1.90955 
2.89787 0.546077 -1.55386 
2.80544 1.55378 -1.19817 
2.71301 2.56148 -0.842473 
2.62058 3.56918 -0.486781 
2.52815 4.57688 -0.131089 
1.4414 4.60837 0.489268 
0.354637 4.63985 1.10963 
-0.732122 4.67134 1.72998 
-1.81888 4.70282 2.35034 
-0.782972 4.2846 3.01774 
0.252937 3.86637 -2.59805 
1.3647 4.07478 2.9602 
2.47647 4.28319 2.23526 
3.58823 4.49159 1.51033 
4.7 4.7 0.785398 

];

ll = [-0.15;-0.15;1];
lr = [0.15;-0.15;1];
ul = [-0.15;0.15;1];
ur = [0.15;0.15;1];

for i = 1:size(p,1)
    x = p(i,1);
    y = p(i,2);
    theta = p(i,3);
    rtMat = [cos(theta), -sin(theta),x;
        sin(theta),cos(theta), y;
        0,0,1];
    newll = (rtMat*ll)';
    newlr = (rtMat*lr)';
    newul = (rtMat*ul)';
    newur = (rtMat*ur)';
    %[newll;newlr;newul;newur]    
    plot([newll(1), newlr(1)],[newll(2),newlr(2)], 'r', ...
        [newll(1), newul(1)], [newll(2),newul(2)], 'r', ...
        [newul(1), newur(1)],[newul(2),newur(2)], 'r', ...
        [newlr(1), newur(1)], [newlr(2),newur(2)], 'r');
end
%plot([0.86228, 4.7],[-4.43988, 4.7], 'r');
title('Environment 2, square robot');
hold off   
