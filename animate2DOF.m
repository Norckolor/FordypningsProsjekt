function animate2DOF(parms,theta1,theta2,dt,varargin)
%ANIMATE2DOF  Animate a planar 2‑link manipulator.
%
%   animate2DOF(L1,L2,theta1,theta2,dt) 
%       L1, L2   – link lengths (scalar, >0)  
%       theta1   – vector of joint‑1 angles [rad] (size N×1)  
%       theta2   – vector of joint‑2 angles [rad] (size N×1)  
%       dt       – time step between successive frames [s] (scalar)
%
%   Optional name‑value pairs (entered after dt):
%       'Trace'      – true/false, whether to draw the end‑effector trace (default: true)
%       'FigureName' – string, title of the figure window (default: '2‑DOF Robot')
%       'SaveVideo'  – struct with fields:
%                         .filename (string, e.g. 'robot.mp4')
%                         .fps      (numeric, frames per second)
%                     – when supplied, the animation is also written to a video file.
%
%   Example
%   -------
%   t  = linspace(0,10,200);
%   th1 = 0.5*sin(0.6*t) + pi/4;
%   th2 = 0.8*cos(0.9*t) - pi/6;
%   animate2DOF(1.0,0.8,th1,th2,0.05);
%
%   See also: plot, animatedline, VideoWriter
% -------------------------------------------------------------------------
% 1.  Parse and check inputs
% -------------------------------------------------------------------------
% parameters: 
J1 = parms.J1;
J2 = parms.J2;
L1 = parms.L1;
L2 = parms.L2;
m1 = parms.m1;
m2 = parms.m2;
g = parms.g;

n = numel(theta1);
assert(numel(theta2)==n,'theta1 and theta2 must have the same length');
assert(isscalar(dt) && dt>0,'dt must be a positive scalar');

% Default optional arguments
traceFlag   = true;
figName     = '2‑DOF Robot';
saveVideo   = struct('filename','', 'fps',30);   % empty filename ⇒ no video

% ----- process name‑value pairs -----
k = 1;
while k <= numel(varargin)
    switch lower(varargin{k})
        case 'trace'
            traceFlag = logical(varargin{k+1});
        case 'figurename'
            figName = varargin{k+1};
        case 'savevideo'
            sv = varargin{k+1};
            if isfield(sv,'filename'), saveVideo.filename = sv.filename; end
            if isfield(sv,'fps'),      saveVideo.fps      = sv.fps;      end
        otherwise
            error('Unknown name‑value pair: %s', varargin{k});
    end
    k = k + 2;
end

% -------------------------------------------------------------------------
% 2.  Prepare the figure
% -------------------------------------------------------------------------
figure('Color','w','Name',figName);
axis equal; grid on; hold on;
reach = L1+L2;
axis([-reach reach -reach reach]);

% Base marker
plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');

% Pre‑allocate line/point objects (fast update)
hLink1 = plot([0 0],[0 0],'b-','LineWidth',4);
hLink2 = plot([0 0],[0 0],'r-','LineWidth',4);
hJoint1 = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
hJoint2 = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
hEE    = plot(0,0,'kp','MarkerSize',12,'MarkerFaceColor','g');

if traceFlag
    hTrace = animatedline('Color','g','LineWidth',1.5);
end

% -------------------------------------------------------------------------
% 3.  (Optional) set up video writer
% -------------------------------------------------------------------------
if ~isempty(saveVideo.filename)
    vid = VideoWriter(saveVideo.filename,'MPEG-4');
    vid.FrameRate = saveVideo.fps;
    open(vid);
end

% -------------------------------------------------------------------------
% 4.  Animation loop
% -------------------------------------------------------------------------
for k = 1:n
    th1 = theta1(k);
    th2 = theta2(k);
    
    % ----- forward kinematics -----
    % Joint 1 (end of link 1)
    x1 = L1*cos(th1);
    y1 = L1*sin(th1);
    % End‑effector (end of link 2)
    x2 = x1 + L2*cos(th1+th2);
    y2 = y1 + L2*sin(th1+th2);
    
    % ----- update graphics -----
    set(hLink1,'XData',[0 x1],'YData',[0 y1]);
    set(hLink2,'XData',[x1 x2],'YData',[y1 y2]);
    set(hJoint1,'XData',x1,'YData',y1);
    set(hJoint2,'XData',x2,'YData',y2);
    set(hEE,'XData',x2,'YData',y2);
    
    if traceFlag
        addpoints(hTrace,x2,y2);
    end
    
    drawnow limitrate               % smooth rendering
    
    % ----- write video frame (if requested) -----
    if ~isempty(saveVideo.filename)
        frame = getframe(gcf);
        writeVideo(vid,frame);
    end
    
    % Respect the user‑specified time step (only for real‑time feel)
    pause(dt);
end

% -------------------------------------------------------------------------
% 5.  Clean‑up
% -------------------------------------------------------------------------
if ~isempty(saveVideo.filename)
    close(vid);
    fprintf('Video saved as %s\n',saveVideo.filename);
end
end
