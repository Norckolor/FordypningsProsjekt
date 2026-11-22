function Dp = Dp(x,s,A,parms)
    F = [parms.Path_obj.dh(s);
        parms.Path_obj.ddh(s)];
    
    xs = [parms.Path_obj.qr(s);
          parms.Path_obj.dqr(s)];
    
    
    %only added for structure is e=0 when function is in use;
    e = x - xs;
    dF = 0; %this does not matter since i only use this function when x-xs = 0
    Dp = (F'*A)/(F'*A*F - e'*A*dF);


end