function vel = controlP4(goal, q, d, dt, first)

% Constants
Kh = 0.04;
Kv = 0.015;
Ki = 0.003;

% Setup e value
persistent e_int;
if first || isempty(e_int)
    e_int = 0;
end

% Get values
x_goal = goal(1);
y_goal = goal(2);
x = q(1);
y = q(2);
theta = q(3);

% Calculate angle difference
theta_goal = atan2(y_goal-y, x_goal-x);
h = theta_goal - theta;
while (h >  pi); h = h-2*pi; end
while (h < -pi); h = h+2*pi; end

% Calculate turning speed
w = Kh * h;
w = min(w, 1);
w = max(w, -1);

% Calculate forward speed
e = sqrt((x_goal-x)^2 + (y_goal-y)^2) - d;
e_int = e_int + e * dt;
v = Kv * e + Ki * e_int;
v = min(v, 0.5);

% Calculate wheel speeds
vw = [v w];
vel = vw2wheels(vw);
disp(vel);

end