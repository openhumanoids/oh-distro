function makeFist

% sends a command to the sandia hands to make a strong fist (e.g., useful
% for crawling, getting up from prone, etc)

[r_left,r_right] = createSandiaManips();
lc = lcm.lcm.LCM.getSingleton();

Kp = repmat([80 80 80],1,4);
Kd = repmat([.1 .1 .1],1,4);

t = 0;
q = [0 pi/2 pi/2   0 pi/2 pi/2  0 pi/2 pi/2  0 1.3 0];
ff = zeros(1,12);
data = [Kp,Kd,q,ff];

nq = getNumDOF(r_left);
joint_names = r_left.getStateFrame.coordinates(1:nq);
coder_left = JLCMCoder(SandiaJointCommandCoder('atlas',false,'left',joint_names));
msg = coder_left.encode(t,data);
lc.publish('L_HAND_JOINT_COMMANDS',msg);

nq = getNumDOF(r_right);
joint_names = r_right.getStateFrame.coordinates(1:nq);
coder_right = JLCMCoder(SandiaJointCommandCoder('atlas',false,'right',joint_names));
msg = coder_right.encode(t,data);
lc.publish('R_HAND_JOINT_COMMANDS',msg);
