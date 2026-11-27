function [k,Rfun,info,C0v,Civ,Div] = MakeController(parms)

xs = @(s) [parms.Path_obj.qr(s);
      parms.Path_obj.dqr(s)];

A = eye(4);
As = @(s) Df(xs(s),s,parms);
Bs = @(s) [0 0;
           0 0;
           inv(M(xs(s),parms))];

F = @(s) [parms.Path_obj.dh(s);
          parms.Path_obj.ddh(s)];
P = @(s) Dp(xs(s),s,A,parms);
eps = @(s) eye(4) - F(s)*P(s);
f_ = @(s) F(s)*parms.Path_obj.rho(s);


%function variables:
sT = 4;
n = 4;
m = 2;
order = 50;
l = 100;
fnc.rho = @(s) parms.Path_obj.rho(s);
fnc.F = @(s) F(s);
fnc.P = @(s) P(s);

fnc.Aperp = @(s) As(s) - ((f_(s)*f_(s)')/(f_(s)'*A*f_(s))) *(A*As(s)+As(s)'*A);

fnc.Bperp = @(s) eps(s) * Bs(s);


[k,Rfun,info,C0v,Civ,Div] = prjprde_sdp(sT,n,m,order,l,fnc);

end