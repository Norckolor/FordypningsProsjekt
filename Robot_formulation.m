function [M,C,G,f] = Robot_formulation(m,p)
    n = size(m,1);

    syms q [1 n]
    syms dq [1 n]
    syms dq [1 n]

    K = 1/2 * dq.'*m*dq;

    L = K - p(q);




    M = m;
end
