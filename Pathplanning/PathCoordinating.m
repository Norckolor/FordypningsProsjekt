function [p] = PathCoordinating(s,h)
    N = size(h,2);
    n = size(h,1);

    j = floor(mod(s,N)) + 1;

    p = zeros(n,1);
    for i = 1:n
        temp = h{i,j};
        p(i) = temp(s-floor(s));
    end
end