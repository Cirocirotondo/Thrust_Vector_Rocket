figure;

% Create a large subplot spanning rows 1 to 3
subplot(5, 1, 1:3);
plot(0:0.1:10, sin(0:0.1:10), 'r');
title('Large Plot Spanning Rows 1-3');
xlabel('X');
ylabel('Y');
grid on;

% Create a small subplot in row 4
subplot(5, 1, 4);
plot(0:0.1:10, cos(0:0.1:10), 'b');
title('Small Plot in Row 4');
xlabel('X');
ylabel('Y');
grid on;

% Create another small subplot in row 5
subplot(5, 1, 5);
plot(0:0.1:10, tan(0:0.1:10), 'g');
title('Small Plot in Row 5');
xlabel('X');
ylabel('Y');
grid on;