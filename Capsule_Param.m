%% capsule_param.m
% Parameters for the payload capsule pitch and yaw models.
% Run this before the Simulink model or capsule_design.m.
%
% The texts used for reference are Performance, Stability, Dynamics, and
% Control of Airplanes by Pomadi and James S. Barrowman's seminal 1966 
% NASA report
%
% Units: SI inside the model (m, kg, s, rad). Inputs in inches are converted.
% Replace every value marked "UPDATE" with your measured or CAD numbers.
%
% Nomenclature
%   L, D, R   body length, diameter, radius             [m]
%   S         reference area = pi*D^2/4                  [m^2]
%   m, Iyy    mass and pitch moment of inertia           [kg, kg*m^2]
%   Izz       yaw moment of inertia                       [kg*m^2]
%   x_cg      CG distance from nose tip                  [m]
%   alpha     angle of attack                             [rad]
%   beta      sideslip angle                              [rad]
%   q, r      pitch and yaw rates                         [rad/s]
%   delta     movable fin deflection                      [rad]
%   V         airspeed                                   [m/s]
%   qbar      dynamic pressure = 0.5*rho*V^2             [Pa]
%   CNa       normal-force slope (per rad, ref area S)
%   Cma       pitch-moment slope about the CG (ref length D)
%   Cmq       pitch damping, per unit of q*D/(2V)
%   Cmd       pitch moment per rad of fin deflection
%
in2m = 0.0254; % conversion of inches to meter

%% Geometry (Design all done via SolidWorks2025)
L    = 14.0 * in2m;     % body length
D    = 2.4  * in2m;     % body diameter
R    = D/2;
S    = pi*D^2/4;
Ln   = 3.383  * in2m;     % nose cone length

% Fixed forward wings (one opposing pair, for pitch)
Nw        = 2;
bw        = 8.2 * in2m;       % total tip-to-tip span, including body
cr_w      = 2.72 * in2m;      % root chord
ct_w      = 1.85 * in2m;      % tip chord
sw        = (bw - D)/2;       % exposed span of one wing
sweep_w   = 24.62 * pi/180;   % leading-edge sweep
x_wing_le = 6.34 * in2m;     % root leading edge from nose tip

% Tail fins (one movable pair, for pitch)
cr    = 2.4 * in2m;     % root chord
ct    = 1.2 * in2m;     % tip chord
sf    = 1.262 * in2m;   % span of one fin, root to tip
sweep = 51.36 * pi/180; % leading-edge sweep
pivot = 1.28 * in2m;    % point of pivot
x_fin_le = 11.81 * in2m; % root leading edge from nose tip

%% Mass properties
m    = 0.7938;     % [kg]
x_cg = 0.465 * L;  % [m]
Iyy  = 0.006105;   % [kg*m^2]
Izz  = 0.006251;   % [kg*m^2]
%% Air
rho = 1.225;            % [kg/m^3] sea level
g   = 9.81;             % [m/s^2]

%% Barrowman estimates (subsonic, small alpha) via Pomadi
% Nose cone (ogive or cone): CNa = 2, center of pressure ~0.466*Ln (ogive)
CNa_nose = 2.0;
X_nose   = 0.466 * Ln;

% Fixed forward wings: one pair in the pitch plane (Nw = 2)
xr_w     = sw * tan(sweep_w);                         % tip LE offset aft of root LE
lf_w     = hypot(sw, (xr_w + ct_w/2) - cr_w/2);       % mid-chord line length
CNa_wing = (4*Nw*(sw/D)^2) / (1 + sqrt(1 + (2*lf_w/(cr_w+ct_w))^2));
K_fb_w   = 1 + R/(sw + R);                           % wing-body interference
CNa_wing = K_fb_w * CNa_wing;
X_wing   = x_wing_le + xr_w*(cr_w + 2*ct_w)/(3*(cr_w + ct_w)) ...
           + (1/6)*(cr_w + ct_w - cr_w*ct_w/(cr_w + ct_w));

% Fins: one pair in the pitch plane (N = 2)
N    = 2;
xr   = sf * tan(sweep);                       % tip LE offset aft of root LE
lf   = hypot(sf, (xr + ct/2) - cr/2);         % mid-chord line length
CNa_fin = (4*N*(sf/D)^2) / (1 + sqrt(1 + (2*lf/(cr+ct))^2));
K_fb    = 1 + R/(sf + R);                     % fin-body interference
CNa_fin = K_fb * CNa_fin;
X_fin   = x_fin_le + xr*(cr + 2*ct)/(3*(cr + ct)) ...
          + (1/6)*(cr + ct - cr*ct/(cr + ct));

% Whole vehicle (nose + fixed wings + movable rear fins)
% Assumes both opposing pairs act in the pitch plane; verify fin orientation.
CNa   = CNa_nose + CNa_wing + CNa_fin;
X_cp  = (CNa_nose*X_nose + CNa_wing*X_wing + CNa_fin*X_fin) / CNa;
SM    = (X_cp - x_cg) / D;                    % static margin [calibers]

% Moment derivatives about the CG (reference length D)
l_w  = (X_wing - x_cg) / D;                   % forward wing arm in calibers
l_t  = (X_fin - x_cg) / D;                    % fin arm in calibers
Cma  = -CNa * SM;
Cmq  = -2 * (CNa_wing*l_w^2 + CNa_fin*l_t^2); % wing/fin damping estimate
CNd  = 0.8 * CNa_fin;                         % all-moving fin effectiveness (guess; UPDATE with CFD)
Cmd  = -CNd * l_t;                            % +delta gives nose-down (negative) moment

fprintf('Static margin = %.2f calibers (target 1-2)\n', SM);
fprintf('X_cp = %.3f m, x_cg = %.3f m\n', X_cp, x_cg);
fprintf('X_wing = %.3f m, CNa_wing = %.2f per rad\n', X_wing, CNa_wing);
fprintf('Cma = %.2f, Cmq = %.1f, Cmd = %.2f per rad\n', Cma, Cmq, Cmd);

%% Servo: GDW DS031MG at 5 V, direct drive (1:1)
gear     = 1.0;                         % fin deg per servo deg       UPDATE if WE CHOOSE TO GEAR
tau_s    = 0.03;                        % first-order lag [s] time const
rateMax  = gear * (60*pi/180)/0.13;     % fin rate limit [rad/s] (0.13 s/60 deg at 4.8 V)
dMax     = 15*pi/180;                   % fin travel limit [rad]

%% Controller timing
Ts = 0.01;                              % 100 Hz (Nano IMU rate)

%% Linear pitch models over a speed grid
% States x = [alpha; q], input u = delta (fin angle, after the servo)
% Capsule-only aerodynamic approximation; parachute loads not included.
V_grid = [5 10 15 20 25 30];            % [m/s]
nV = numel(V_grid);
A_all = zeros(2,2,nV);
B_all = zeros(2,1,nV);
for i = 1:nV
    V    = V_grid(i);
    qbar = 0.5*rho*V^2;
    A_all(:,:,i) = [ -qbar*S*CNa/(m*V),        1 ;
                      qbar*S*D*Cma/Iyy,  qbar*S*D*Cmq*(D/(2*V))/Iyy ];
    B_all(:,:,i) = [ -qbar*S*CNd/(m*V) ;
                      qbar*S*D*Cmd/Iyy ];
end

%% Pick the design point used by the Simulink model
V_sel = 20;                                  % [m/s]   change to test other speeds
iSel  = find(V_grid == V_sel, 1);
assert(~isempty(iSel), 'V_sel must be included in V_grid.');
A_sel = A_all(:,:,iSel);
B_sel = B_all(:,:,iSel);
C_sel = eye(2);
D_sel = [0; 0];

alpha0 = 20*pi/180;                          % initial pointing error for tests
x0     = [alpha0; 0];

% Placeholder gains, so the model runs before capsule_design.m is used
if ~exist('Kp','var'), Kp = 0.5; Ki = 0.5; Kd = 0.05; Nf = 50; end

