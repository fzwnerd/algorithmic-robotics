% obstacles
rectangle('Position', [0.5 0 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-1 -2 2 1], 'FaceColor',[0 .5 .5]);
rectangle('Position', [-3 -2.5 1 5], 'FaceColor',[0 .5 .5]);
rectangle('Position', [3.35 -1.25 0.5 3.75], 'FaceColor',[0 .5 .5]);
hold on
% robot
p1 = [-0.28 1.49716];
p2 = [-0.562843 1.78];
p3 = [0.00284271 1.78];
p4 = [-0.28 2.06284];
plot([p1(1) p2(1)], [p1(2) p2(2)], 'Color', [1 0 0]);
plot([p1(1) p3(1)], [p1(2) p3(2)], 'Color', [1 0 0]);
plot([p1(1) p4(1)], [p1(2) p4(2)], 'Color', [1 0 0]);
plot([p2(1) p3(1)], [p2(2) p3(2)], 'Color', [1 0 0]);
plot([p2(1) p4(1)], [p2(2) p4(2)], 'Color', [1 0 0]);
plot([p3(1) p4(1)], [p3(2) p4(2)], 'Color', [1 0 0]);
hold off