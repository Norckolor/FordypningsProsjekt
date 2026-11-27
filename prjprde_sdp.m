function [K, Rfun, info] = prjprde_sdp(sT, n, m, order, l, fcn, opts)
% PRJPRDE_SDP  Approximate solution of the projected periodic Riccati
% differential equation (PrjPRDE, eq. 4.96) via the SDP (4.99)
% (Saetre thesis, Sec. 4.6.3), using YALMIP.
%
%   max   sum_j  Tr(E'(s_j) R(s_j) E(s_j)) - F'(s_j) R(s_j) F(s_j)
%   s.t.  RSC(R(s_j), s_j) >= 0,   R(s_j) >= 0,      j = 0..l
%
%   R(s) = C0 + sum_{i=1}^{order} [Ci sin(2*i*pi*s/sT) + Di cos(2*i*pi*s/sT)]
%
%   RSC = [ E'(rho R' + A'R + R A + Q + kappa R)E ,  E' R B ;
%           B' R E                                 ,  inv(Gamma) ]
%
% INPUTS
%   sT     period of the s-parameterization (S = [0,sT))
%   n, m   state / input dimension
%   order  order of the trigonometric polynomial
%   l      grid points s_0=0 < ... < s_l < sT   (l+1 points, evenly spaced)
%   fcn    struct of function handles (all take scalar s):
%            fcn.rho(s)    -> scalar   (s-dot = rho(s) > 0)
%            fcn.F(s)      -> n x 1    (d xs / ds)
%            fcn.P(s)      -> 1 x n    (Dp(xs(s)),  P*F = 1)
%            fcn.Aperp(s)  -> n x n    (A_perp, eq. 4.56a)
%            fcn.Bperp(s)  -> n x m    (B_perp = E*Bs, eq. 4.56b)
%   opts   struct (all optional):
%            opts.Q      handle s->n x n PSD   (default @(s) eye(n))
%            opts.Gamma  m x m PD              (default eye(m))
%            opts.kappa  handle s->scalar>=0   (default @(s) 0)
%            opts.solver e.g. 'sdpt3','mosek','sedumi' (default '')
%            opts.verbose  0/1
%
% OUTPUTS
%   K     function handle s -> m x n,  K(s) = -Gamma*Bperp'(s)*R(s)  (4.97)
%   Rfun  function handle s -> n x n,  the found R(s)
%   info  YALMIP diagnostics + max constraint residuals

if nargin < 7, opts = struct; end
if ~isfield(opts,'Q'),      opts.Q = @(s) eye(n); end
if ~isfield(opts,'Gamma'),  opts.Gamma = eye(m);  end
if ~isfield(opts,'kappa'),  opts.kappa = @(s) 0;  end
if ~isfield(opts,'solver'), opts.solver = '';     end
if ~isfield(opts,'verbose'),opts.verbose = 0;     end

yalmip('clear');

% ---- Decision variables: symmetric coefficient matrices -------------
C0 = sdpvar(n,n,'symmetric');
Ci = cell(order,1); Di = cell(order,1);
for i = 1:order
    Ci{i} = sdpvar(n,n,'symmetric');
    Di{i} = sdpvar(n,n,'symmetric');
end

% R(s) and R'(s) as functions returning sdpvar expressions
Rexpr  = @(s) C0 + sumterms(Ci,Di,order,sT,s,0);
dRexpr = @(s) sumterms(Ci,Di,order,sT,s,1);

% ---- Grid ------------------------------------------------------------
sgrid = (0:l) * sT/(l+1);          % s_l < sT as required
Gi    = inv(opts.Gamma);

cons = [];
obj  = 0;
for j = 1:numel(sgrid)
    s   = sgrid(j);
    rho = fcn.rho(s);
    F   = fcn.F(s);
    P   = fcn.P(s);
    A   = fcn.Aperp(s);
    B   = fcn.Bperp(s);
    Q   = opts.Q(s);
    kap = opts.kappa(s);
    E   = eye(n) - F*P;            % E_perp(s) = I - F P

    R  = Rexpr(s);
    dR = dRexpr(s);

    M11 = E' * (rho*dR + A'*R + R*A + Q + kap*R) * E;
    M12 = E' * R * B;
    RSC = [M11, M12; M12', Gi];
    RSC = (RSC + RSC')/2;          % enforce exact symmetry for YALMIP

    cons = [cons, RSC >= 0, R >= 0];                 
    obj  = obj + trace(E'*R*E) - F'*R*F;             
end

% ---- Solve (YALMIP minimizes, so negate the objective) --------------
ops = sdpsettings('verbose',opts.verbose);
if ~isempty(opts.solver), ops = sdpsettings(ops,'solver',opts.solver); end
sol = optimize(cons, -obj, ops);

% ---- Extract numeric solution ---------------------------------------
C0v = value(C0);
Civ = cellfun(@value, Ci, 'UniformOutput', false);
Div = cellfun(@value, Di, 'UniformOutput', false);

Rfun = @(s) C0v + sumterms_num(Civ,Div,order,sT,s,0);
dRf  = @(s) sumterms_num(Civ,Div,order,sT,s,1);
K    = @(s) -opts.Gamma * fcn.Bperp(s)' * Rfun(s);

% ---- Residual of the PrjPRDE (4.96) on a finer grid ------------------
sfine = linspace(0, sT, 4*(l+1)+1);
res = 0; mineig = inf;
for s = sfine
    rho = fcn.rho(s); F = fcn.F(s); P = fcn.P(s);
    A = fcn.Aperp(s); B = fcn.Bperp(s); E = eye(n) - F*P;
    R = Rfun(s);
    LHS = E'*(rho*dRf(s) + A'*R + R*A + opts.Q(s) + opts.kappa(s)*R ...
          - R*B*opts.Gamma*B'*R)*E;
    res = max(res, norm(LHS,'fro'));
    mineig = min(mineig, min(eig((R+R')/2)));
end

info.yalmip      = sol;
info.problem     = sol.problem;            % 0 = solved OK
info.info        = sol.info;
info.max_residual = res;                   % ||PrjPRDE LHS||_F, want ~ 0
info.min_eig_R    = mineig;                % want >= 0
end

% ====================================================================
function S = sumterms(Ci,Di,order,sT,s,der)
% symbolic (sdpvar) trig sum; der=1 returns d/ds
S = 0;
for i = 1:order
    w = 2*i*pi/sT;
    if der == 0
        S = S + Ci{i}*sin(w*s) + Di{i}*cos(w*s);
    else
        S = S + w*(Ci{i}*cos(w*s) - Di{i}*sin(w*s));
    end
end
end

function S = sumterms_num(Civ,Div,order,sT,s,der)
S = zeros(size(Civ{1}));
for i = 1:order
    w = 2*i*pi/sT;
    if der == 0
        S = S + Civ{i}*sin(w*s) + Div{i}*cos(w*s);
    else
        S = S + w*(Civ{i}*cos(w*s) - Div{i}*sin(w*s));
    end
end
end
