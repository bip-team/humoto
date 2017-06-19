clear
source ('./test_003_feasibility.m')

feasibility_flags = (feasibility_flags==-1);

figure
hold on
%{
drop = (points(:,3) < 0.75);
feasibility_flags(drop) = false;
drop = (points(:,3) > 0.76);
feasibility_flags(drop) = false;
%}

plot3(points(feasibility_flags, 1), points(feasibility_flags, 2), points(feasibility_flags, 3), '.b', 'MarkerSize', 15)
%plot3(points(not(feasibility_flags), 1), points(not(feasibility_flags), 2), points(not(feasibility_flags), 3), '.r', 'MarkerSize', 15)
hold off
xlabel('x'); ylabel('y'); zlabel('z');
%view([180,90]);
