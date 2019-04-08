goal = [2;2];
q = [0 0 -3*pi/4];
d = 0.15;
dt = 0.2;
first = true;
control.purePursuit(goal, q, d, dt, 0)