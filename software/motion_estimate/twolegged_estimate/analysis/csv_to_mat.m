function state = csv_to_mat(csv_file, mat_file)

data = load(csv_file);

% check if kinematic foot velocities are available
legkinvels = size(data,2)>91;

t = data(:,25);

Ts = mean(diff(t))*1E-9/5;

t = linspace(0,(t(end)-t(1))*Ts,length(t));

tpos = data(:,1:3);
tvel = data(:,4:6);
tE = data(:,7:9);
trate = data(:,10:12);

epos = data(:,13:15);
evel = data(:,16:18);
eE = data(:,19:21);
erate = data(:,22:24);

left_con = data(:,29);
right_con = data(:,30);


%joint commands

offset = 31;

ce_l_uhz = data(:,(offset+4));
ce_l_mhx = data(:,(offset+5));
ce_l_lhy = data(:,(offset+6));
ce_l_kny = data(:,(offset+7));
ce_l_uay = data(:,(offset+8));
ce_l_lax = data(:,(offset+9));
ce_r_uhz = data(:,(offset+10));
ce_r_mhx = data(:,(offset+11));
ce_r_lhy = data(:,(offset+12));
ce_r_kny = data(:,(offset+13));
ce_r_uay = data(:,(offset+14));
ce_r_lax = data(:,(offset+15));

% offset = offset+28;
offset = 59;

jp_l_uhz = data(:,(offset+4));
jp_l_mhx = data(:,(offset+5));
jp_l_lhy = data(:,(offset+6));
jp_l_kny = data(:,(offset+7));
jp_l_uay = data(:,(offset+8));
jp_l_lax = data(:,(offset+9));
jp_r_uhz = data(:,(offset+10));
jp_r_mhx = data(:,(offset+11));
jp_r_lhy = data(:,(offset+12));
jp_r_kny = data(:,(offset+13));
jp_r_uay = data(:,(offset+14));
jp_r_lax = data(:,(offset+15));

% offset = offset+16;
offset = 75;

% me_l_uhz = data(:,(offset+4));
% me_l_mhx = data(:,(offset+5));
% me_l_lhy = data(:,(offset+6));
%
% me_l_lax = data(:,(offset+9));
% me_r_lax = data(:,(offset+15));

me_l_uhz = data(:,(offset+4));
me_l_mhx = data(:,(offset+5));
me_l_lhy = data(:,(offset+6));
me_l_kny = data(:,(offset+7));
me_l_uay = data(:,(offset+8));
me_l_lax = data(:,(offset+9));
me_r_uhz = data(:,(offset+10));
me_r_mhx = data(:,(offset+11));
me_r_lhy = data(:,(offset+12));
me_r_kny = data(:,(offset+13));
me_r_uay = data(:,(offset+14));
me_r_lax = data(:,(offset+15));

% 	  ss << joint_commands[i] << ", "; //31-58
% 		ss << joint_positions[i] << ", "; //59-74
% 	   ss << measured_joint_effort[i] << ", ";//75-90

% if (legkinvels)
%foot velocities are available

offset = 91;

l_foot_velx = data(:,(offset));
l_foot_vely = data(:,(offset+1));
l_foot_velz = data(:,(offset+2));

r_foot_velx = data(:,(offset+3));
r_foot_vely = data(:,(offset+4));
r_foot_velz = data(:,(offset+5));

l_foot_ratex = data(:,(offset+6));
l_foot_ratey = data(:,(offset+7));
l_foot_ratez = data(:,(offset+8));

r_foot_ratex = data(:,(offset+9));
r_foot_ratey = data(:,(offset+10));
r_foot_ratez = data(:,(offset+11));


jv_l_uhz = [0;diff(jp_l_uhz)*1E3];
jv_l_mhx = [0;diff(jp_l_mhx)*1E3];
jv_l_lhy = [0;diff(jp_l_lhy)*1E3];
jv_l_kny = [0;diff(jp_l_kny)*1E3];
jv_l_uay = [0;diff(jp_l_uay)*1E3];
jv_l_lax = [0;diff(jp_l_lax)*1E3];
jv_r_uhz = [0;diff(jp_r_uhz)*1E3];
jv_r_mhx = [0;diff(jp_r_mhx)*1E3];
jv_r_lhy = [0;diff(jp_r_lhy)*1E3];
jv_r_kny = [0;diff(jp_r_kny)*1E3];
jv_r_uay = [0;diff(jp_r_uay)*1E3];
jv_r_lax = [0;diff(jp_r_lax)*1E3];


offset = 103;

fjp_l_uhz = data(:,(offset+4));
fjp_l_mhx = data(:,(offset+5));
fjp_l_lhy = data(:,(offset+6));
fjp_l_kny = data(:,(offset+7));
fjp_l_uay = data(:,(offset+8));
fjp_l_lax = data(:,(offset+9));
fjp_r_uhz = data(:,(offset+10));
fjp_r_mhx = data(:,(offset+11));
fjp_r_lhy = data(:,(offset+12));
fjp_r_kny = data(:,(offset+13));
fjp_r_uay = data(:,(offset+14));
fjp_r_lax = data(:,(offset+15));

offset = 119;
stageA = data(:,(offset:(offset+2)));
offset = 122;
stageB = data(:,(offset:(offset+2)));
offset = 125;
stageC = data(:,(offset:(offset+2)));

offset = 128;
imu_P = data(:,(offset:(offset+2)));
offset = 131;
imu_V = data(:,(offset:(offset+2)));

offset = 134;
leg_V = data(:,(offset:(offset+2)));

offset = 137;
fovis_V = data(:,(offset:(offset+2)));




% end


% form structure
all_vars = who;
state = struct();
for i = 1:numel(all_vars)
    state.(all_vars{i}) = eval(all_vars{i});
end

% save to file if necessary
if (exist('mat_file','var') && numel(mat_file)>0)
    save(mat_file);
end
