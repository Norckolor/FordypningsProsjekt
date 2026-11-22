function plots(parms,y,dt,t)
    %% calculate vales for plots
    Path_obj = parms.Path_obj;
    n = Path_obj.dofs;
    ds = dt;
    qr = @(s) Path_obj.qr(s);
    dqr = @(s) Path_obj.dqr(s);
    ddqr = @(s) Path_obj.ddqr(s);
    
    q = zeros(n,length(t));
    dq = zeros(n,length(t));
    ddq = zeros(n,length(t));
    Jerk = zeros(n,length(t));
    
    ddq_last = zeros(n,1);
    for k = 1:length(t)
        q(:,k) = qr(t(k));
        dq(:,k) = dqr(t(k));
        ddq(:,k) = ddqr(t(k));
        Jerk(:,k) = (ddq(:,k) - ddq_last)/ds;
        ddq_last = ddq(:,k);
    end
    
    %% phase plane
    %%% plot phase planes:
    figure name 'q dq'
    subplot(2,1,1);
    hold on
    grid on
    title('phase plane nominal q_1')
    plot(q(1,:),dq(1,:),LineWidth=1.5);
    xlabel('\(q_1\)','Interpreter','latex');
    ylabel('\(\dot{q}_1\)','Interpreter','latex',Rotation=0);
    hold off
    
    subplot(2,1,2);
    hold on
    grid on
    title('phase plane nominal q_2')
    plot(q(2,:),dq(2,:),LineWidth=1.5);
    xlabel('\(q_2\)','Interpreter','latex');
    ylabel('\(\dot{q}_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %% desired orbits
    %%% plot orbit qr in state space
    figure name 'qr orbit in state space'
    q_mid = Path_obj.qr_mid;
    hold on
    grid on
    title('orbit in q_1, q_2 space')
    plot(q(1,:),q(2,:),LineWidth=1.5);
    plot(q_mid(1),q_mid(2),'p')
    plot(q(1,1),q(2,1),'p')
    xlabel('\(q_1\)','Interpreter','latex');
    ylabel('\(q_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %%% plot orbit dqr in state space
    figure name 'dqr orbit in state space'
    q_mid = Path_obj.qr_mid;
    hold on
    grid on
    title('orbit in q_1, q_2 space')
    plot(dq(1,:),dq(2,:),LineWidth=1.5);
    plot(dq(1,1),dq(2,1),'p')
    xlabel('\(q_1\)','Interpreter','latex');
    ylabel('\(q_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %%% plot orbit ddqr in state space
    figure name 'ddqr orbit in state space'
    hold on
    grid on
    title ('desired path ddq')
    plot(ddq(1,:),ddq(2,:),LineWidth=2.5);
    plot(ddq(1,1),ddq(2,1),'p')
    xlabel('\(\ddot{q}_1\)','Interpreter','latex');
    ylabel('\(\ddot{q}_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %% actual path taken
    %%% plot desired path and actual path
    figure name 'walked path q'
    hold on
    grid on
    title ('walked path q')
    plot(q(1,:),q(2,:),LineWidth=2.5,LineStyle='--');
    plot(y(:,1),y(:,2),LineWidth=1.5);
    plot(q_mid(1),q_mid(2),'p')
    plot(q(1,1),q(2,1),'p')
    plot(q(1,2/dt),q(2,2/dt),'p')
    plot(q(1,3/dt),q(2,3/dt),'p') 
    plot(y(1,1),y(1,2),'p')
    xlabel('\(q_1\)','Interpreter','latex');
    ylabel('\(q_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %%% plot desired path and actual path dq
    figure name 'walked path dq'
    hold on
    grid on
    title ('walked path dq')
    plot(dq(1,:),dq(2,:),LineWidth=2.5,LineStyle='--');
    plot(y(:,3),y(:,4),LineWidth=1.5);
    plot(dq(1,1),dq(2,1),'p')
    plot(y(1,3),y(1,4),'p')
    xlabel('\(\dot{q}_1\)','Interpreter','latex');
    ylabel('\(\dot{q}_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %% desired paths in s
    %%% plot desired path q in s
    figure name 'desired path q in s'
    hold on
    grid on
    title ('desired path q')
    plot(t,q(1,:),LineWidth=2.5);
    plot(t,q(2,:),LineWidth=2.5);
    legend('\(q_1\)','\(q_2\)',Interpreter='latex')
    xlabel('s');
    ylabel('\(q_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %%% plot desired path dq in s
    figure name 'desired path dq in s'
    hold on
    grid on
    title ('desired path dq')
    plot(t,dq(1,:),LineWidth=2.5);
    plot(t,dq(2,:),LineWidth=2.5);
    legend('\(q_1\) velocity','\(q_2\) velocity',Interpreter='latex')
    xlabel('s');
    ylabel('\(\dot{q}_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    
    %%% plot desired path ddq in s
    figure name 'desired path ddq in s'
    hold on
    grid on
    title ('desired path ddq')
    plot(t,ddq(1,:),LineWidth=2.5);
    plot(t,ddq(2,:),LineWidth=2.5);
    legend('\(q_1\) acceleration','\(q_2\) acceleration',Interpreter='latex')
    xlabel('s');
    ylabel('\(\ddot{q}_2\)','Interpreter','latex',Rotation=0);
    hold off
    
    %%% plot desired path jerk in s
    figure name 'desired path jerk in s'
    hold on
    grid on
    title ('desired path jerk')
    plot(t,Jerk(1,:),LineWidth=2.5);
    plot(t,Jerk(2,:),LineWidth=2.5);
    legend('\(q_1\) jerk','\(q_2\) jerk',Interpreter='latex')
    xlabel('s');
    ylabel('\(q^{(3)}_2\)','Interpreter','latex',Rotation=0);
    hold off
end