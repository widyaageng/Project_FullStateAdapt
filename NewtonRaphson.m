
%% Solving non linear equation using Newton raphson
syms x;
f = 0.3679 - exp(-0.31*x) + 0.31*x*exp(-0.31*x);
fdiv = diff(f);

x_prev = 0;
x_next = x_prev - (subs(f,x_prev)/subs(fdiv,x_prev));
while abs(x_prev-x_next)>1e-8
    x_prev = vpa(x_next,8)
    x_next = x_prev - (subs(f,x_prev)/subs(fdiv,x_prev));
end
