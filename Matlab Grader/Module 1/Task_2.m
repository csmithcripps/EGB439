t = linspace(0, 2*pi, 100);

x = (sqrt(2).*cos(t))./(sin(t).^2+1)
y = (sqrt(2).*cos(t).*sin(t))./(sin(t).^2+1)

A = 0.5/max(x);
x = A*x;
y = A*y;
plot(x,y)