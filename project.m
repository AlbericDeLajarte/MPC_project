%% Setup
setup();

%% Part 1
quad = Quad(Ts);

%% Part 6
clc
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL);
quad.plot(sim)

