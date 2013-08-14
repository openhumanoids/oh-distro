function [Kp,Kd] = getPDGains(r,mode)
%NOTEST

if nargin<2
  mode = 'default';
end

fr = getInputFrame(r);
Kp = Point(fr);
Kd = Point(fr);

if strcmpi(mode,'default')
  Kp.l_arm_usy = 625.0; %%%%%
  Kp.l_arm_shx = 3000.0; %%%%%
  Kp.l_arm_ely = 650.0; %%%%%
  Kp.l_arm_elx = 1600.0; %%%%%s
  Kp.l_arm_uwy = 25.0; %%%%%
  Kp.l_arm_mwx = 375.0; %%%%%
  Kp.l_leg_hpz = 800.0; 
  Kp.l_leg_hpx = 6000.0; 
  Kp.l_leg_hpy = 2200.0; 
  Kp.l_leg_kny = 3500.0; 
  Kp.l_leg_aky = 3300.0;
  Kp.l_leg_akx = 3300.0;
  Kp.neck_ay = 40.0; %%%%%
  Kp.back_bkz = 4500.0; % 
  Kp.back_bky = 6000.0; %
  Kp.back_bkx = 2350.0; %

  Kd.l_arm_usy = 38.0; %%%%%
  Kd.l_arm_shx = 75.0; %%%%%
  Kd.l_arm_ely = 30.0; %%%%%
  Kd.l_arm_elx = 60.0; %%%%%
  Kd.l_arm_uwy = 1.0; %%%%%
  Kd.l_arm_mwx = 10.0; %%%%%
  Kd.l_leg_hpz = 55.0; 
  Kd.l_leg_hpx = 95.0; 
  Kd.l_leg_hpy = 50.0; 
  Kd.l_leg_kny = 85.0; 
  Kd.l_leg_aky = 90.0;
  Kd.l_leg_akx = 90.0;  
  Kd.neck_ay = 4.0; %%%%%
  Kd.back_bkz = 50.0; %
  Kd.back_bky = 65.0; %
  Kd.back_bkx = 105.0; %

elseif strcmpi(mode,'pinned')
  Kp.l_arm_usy = 625.0; %%%%%
  Kp.l_arm_shx = 3000.0; %%%%%
  Kp.l_arm_ely = 650.0; %%%%%
  Kp.l_arm_elx = 1600.0; %%%%%
  Kp.l_arm_uwy = 25.0; %%%%%
  Kp.l_arm_mwx = 375.0; %%%%%
  Kp.l_leg_hpz = 250.0; %
  Kp.l_leg_hpx = 350.0; %
  Kp.l_leg_hpy = 400.0; %
  Kp.l_leg_kny = 400.0; %
  Kp.l_leg_aky = 45.0; %
  Kp.l_leg_akx = 45.0; %
  Kp.neck_ay = 40.0; %%%%%
  Kp.back_bkz = 1750.0; % 
  Kp.back_bky = 4200.0; %
  Kp.back_bkx = 2200.0; %

  Kd.l_arm_usy = 38.0; %%%%%
  Kd.l_arm_shx = 75.0; %%%%%
  Kd.l_arm_ely = 30.0; %%%%%
  Kd.l_arm_elx = 60.0; %%%%%
  Kd.l_arm_uwy = 1.0; %%%%%
  Kd.l_arm_mwx = 10.0; %%%%%
  Kd.l_leg_hpz = 30.0; %
  Kd.l_leg_hpx = 60.0; %
  Kd.l_leg_hpy = 60.0; %
  Kd.l_leg_kny = 35.0; %
  Kd.l_leg_aky = 5.0; %
  Kd.l_leg_akx = 5.0; %
  Kd.neck_ay = 4.0; %%%%%
  Kd.back_bkz = 170.0; %
  Kd.back_bky = 300.0; %
  Kd.back_bkx = 250.0; %
  
elseif strcmpi(mode,'pinned2')
    
  Kp.l_arm_usy = 400.0; 
  Kp.l_arm_shx = 2000.0; 
  Kp.l_arm_ely = 200.0; 
  Kp.l_arm_elx = 400.0; 
  Kp.l_arm_uwy = 15.0; 
  Kp.l_arm_mwx = 300.0; 
  Kp.l_leg_hpz = 100.0; 
  Kp.l_leg_hpx = 250.0; 
  Kp.l_leg_hpy = 500.0; 
  Kp.l_leg_kny = 120.0; 
  Kp.l_leg_aky = 15.0;
  Kp.l_leg_akx = 15.0;
  Kp.neck_ay = 100.0; 
  Kp.back_bkz = 5000.0;  
  Kp.back_bky = 3000.0; 
  Kp.back_bkx = 6000.0; 

  Kd.l_arm_usy = 70.0; 
  Kd.l_arm_shx = 70.0; 
  Kd.l_arm_ely = 5.0; 
  Kd.l_arm_elx = 15.0; 
  Kd.l_arm_uwy = 6.0; 
  Kd.l_arm_mwx = 15.0; 
  Kd.l_leg_hpz = 30.0; 
  Kd.l_leg_hpx = 50.0; 
  Kd.l_leg_hpy = 25.0; 
  Kd.l_leg_kny = 5; 
  Kd.l_leg_aky = 2;
  Kd.l_leg_akx = 2;
  Kd.neck_ay = 5.0; 
  Kd.back_bkz = 45.0; 
  Kd.back_bky = 45.0; 
  Kd.back_bkx = 45.0;   
    
elseif strcmpi(mode,'pinned-cecilia');

  Kp.l_arm_usy = 2000.0; 
  Kp.l_arm_shx = 800.0; 
  Kp.l_arm_ely = 200.0; 
  Kp.l_arm_elx = 200.0; 
  Kp.l_arm_uwy = 50.0; 
  Kp.l_arm_mwx = 200.0; 
  Kp.l_leg_hpz = 5.0; 
  Kp.l_leg_hpx = 100.0; 
  Kp.l_leg_hpy = 2000.0; 
  Kp.l_leg_kny = 1000.0; 
  Kp.l_leg_aky = 900.0;
  Kp.l_leg_akx = 300.0;
  Kp.neck_ay = 20.0; 
  Kp.back_bkz = 20.0;  
  Kp.back_bky = 4000.0; 
  Kp.back_bkx = 2000.0; 

  Kd.l_arm_usy = 180.0; 
  Kd.l_arm_shx = 100.0; 
  Kd.l_arm_ely = 4.0; 
  Kd.l_arm_elx = 25.0; 
  Kd.l_arm_uwy = 8.0; 
  Kd.l_arm_mwx = 22.0; 
  Kd.l_leg_hpz = 0.01; 
  Kd.l_leg_hpx = 1.0; 
  Kd.l_leg_hpy = 10.0; 
  Kd.l_leg_kny = 10.0; 
  Kd.l_leg_aky = 8.0;
  Kd.l_leg_akx = 2.0;
  Kd.neck_ay = 1.0; 
  Kd.back_bkz = 0.1; 
  Kd.back_bky = 2.0; 
  Kd.back_bkx = 1.0;    

%   Kp.l_arm_usy = 100.0;  
%   Kp.l_arm_shx = 100.0;  
%   Kp.l_arm_ely = 100.0;   
%   Kp.l_arm_elx = 100.0;   
%   Kp.l_arm_uwy = 100.0;  
%   Kp.l_arm_mwx = 100.0; 
%   Kp.l_leg_hpz = 100.0;  
%   Kp.l_leg_hpx = 100.0;  
%   Kp.l_leg_hpy = 100.0;   
%   Kp.l_leg_kny = 100.0;  
%   Kp.l_leg_aky = 100.0;  
%   Kp.l_leg_akx = 100.0;  
%   Kp.neck_ay = 100.0; 
%   Kp.back_bkz = 100.0;  
%   Kp.back_bky = 100.0;  
%   Kp.back_bkx = 100.0;  
% 
%   Kd.l_arm_usy = 10.0; 
%   Kd.l_arm_shx = 10.0; 
%   Kd.l_arm_ely = 10.0;  
%   Kd.l_arm_elx = 10.0; 
%   Kd.l_arm_uwy = 10.0; 
%   Kd.l_arm_mwx = 10.0; 
%   Kd.l_leg_hpz = 10.0; 
%   Kd.l_leg_hpx = 10.0;  
%   Kd.l_leg_hpy = 10.0;  
%   Kd.l_leg_kny = 10.0; 
%   Kd.l_leg_aky = 10.0; 
%   Kd.l_leg_akx = 10.0; 
%   Kd.neck_ay = 10.0; 
%   Kd.back_bkz = 10.0; 
%   Kd.back_bky = 10.0;  
%   Kd.back_bkx = 10.0; 
  
elseif strcmpi(mode,'stance_leg')
  Kp.l_arm_usy = 625.0; %%%%%
  Kp.l_arm_shx = 3000.0; %%%%%
  Kp.l_arm_ely = 650.0; %%%%%
  Kp.l_arm_elx = 1600.0; %%%%%
  Kp.l_arm_uwy = 25.0; %%%%%
  Kp.l_arm_mwx = 375.0; %%%%%
  Kp.l_leg_hpz = 800.0; 
  Kp.l_leg_hpx = 6200.0; 
  Kp.l_leg_hpy = 2200.0; 
  Kp.l_leg_kny = 3500.0; 
  Kp.l_leg_aky = 3350.0;
  Kp.l_leg_akx = 3300.0;
  Kp.neck_ay = 40.0; %%%%%
  Kp.back_bkz = 4500.0; % 
  Kp.back_bky = 6000.0; %
  Kp.back_bkx = 2350.0; %

  Kd.l_arm_usy = 38.0; %%%%%
  Kd.l_arm_shx = 75.0; %%%%%
  Kd.l_arm_ely = 30.0; %%%%%
  Kd.l_arm_elx = 60.0; %%%%%
  Kd.l_arm_uwy = 1.0; %%%%%
  Kd.l_arm_mwx = 10.0; %%%%%
  Kd.l_leg_hpz = 55.0; 
  Kd.l_leg_hpx = 92.0; 
  Kd.l_leg_hpy = 50.0; 
  Kd.l_leg_kny = 85.0; 
  Kd.l_leg_aky = 100.0;
  Kd.l_leg_akx = 160.0;
  Kd.neck_ay = 4.0; %%%%%
  Kd.back_bkz = 50.0; %
  Kd.back_bky = 65.0; %
  Kd.back_bkx = 105.0; %
    
  
  
elseif strcmpi(mode,'gazebo')
  Kp.l_arm_usy = 2000.0; 
  Kp.l_arm_shx = 1000.0; 
  Kp.l_arm_ely = 200.0; 
  Kp.l_arm_elx = 200.0; 
  Kp.l_arm_uwy = 50.0; 
  Kp.l_arm_mwx = 100.0; 
  Kp.l_leg_hpz = 5.0; 
  Kp.l_leg_hpx = 100.0; 
  Kp.l_leg_hpy = 2000.0; 
  Kp.l_leg_kny = 1000.0; 
  Kp.l_leg_aky = 900.0;
  Kp.l_leg_akx = 300.0;
  Kp.neck_ay = 20.0; 
  Kp.back_bkz = 20.0;  
  Kp.back_bky = 4000.0; 
  Kp.back_bkx = 2000.0; 

  Kd.l_arm_usy = 3.0; 
  Kd.l_arm_shx = 10.0; 
  Kd.l_arm_ely = 3.0; 
  Kd.l_arm_elx = 3.0; 
  Kd.l_arm_uwy = 0.1; 
  Kd.l_arm_mwx = 0.2; 
  Kd.l_leg_hpz = 0.01; 
  Kd.l_leg_hpx = 1.0; 
  Kd.l_leg_hpy = 10.0; 
  Kd.l_leg_kny = 10.0; 
  Kd.l_leg_aky = 8.0;
  Kd.l_leg_akx = 2.0;
  Kd.neck_ay = 1.0; 
  Kd.back_bkz = 0.1; 
  Kd.back_bky = 2.0; 
  Kd.back_bkx = 1.0; 

elseif strcmpi(mode,'bracing')
  Kp.l_arm_usy = 100.0; 
  Kp.l_arm_shx = 100.0; 
  Kp.l_arm_ely = 100.0; 
  Kp.l_arm_elx = 100.0; 
  Kp.l_arm_uwy = 20.0; 
  Kp.l_arm_mwx = 50.0; 
  Kp.l_leg_hpz = 60.0; 
  Kp.l_leg_hpx = 100.0; 
  Kp.l_leg_hpy = 100.0; 
  Kp.l_leg_kny = 100.0; 
  Kp.l_leg_aky = 50.0;
  Kp.l_leg_akx = 50.0;
  Kp.neck_ay = 10.0; 
  Kp.back_bkz = 20.0;  
  Kp.back_bky = 100.0; 
  Kp.back_bkx = 100.0; 

  Kd.l_arm_usy = 15.0; 
  Kd.l_arm_shx = 15.0; 
  Kd.l_arm_ely = 13.0; 
  Kd.l_arm_elx = 13.0; 
  Kd.l_arm_uwy = 0.2; 
  Kd.l_arm_mwx = 0.2; 
  Kd.l_leg_hpz = 5.0; 
  Kd.l_leg_hpx = 5.0; 
  Kd.l_leg_hpy = 10.0; 
  Kd.l_leg_kny = 10.0; 
  Kd.l_leg_aky = 8.0;
  Kd.l_leg_akx = 2.0;
  Kd.neck_ay = 1.0; 
  Kd.back_bkz = 1.0; 
  Kd.back_bky = 2.0; 
  Kd.back_bkx = 1.0; 


elseif strcmpi(mode,'sit_up')
  Kp.l_arm_usy = 60000.0; 
  Kp.l_arm_shx = 60000.0; 
  Kp.l_arm_ely = 60000.0; 
  Kp.l_arm_elx = 60000.0; 
  Kp.l_arm_uwy = 60000.0; 
  Kp.l_arm_mwx = 30000.0; 
  Kp.l_leg_hpz = 30000.0; 
  Kp.l_leg_hpx = 30000.0; 
  Kp.l_leg_hpy = 30000.0; 
  Kp.l_leg_kny = 30000.0; 
  Kp.l_leg_aky = 30000.0;
  Kp.l_leg_akx = 30000.0;
  Kp.neck_ay = 60000.0; 
  Kp.back_bkz = 60000.0;  
  Kp.back_bky = 60000.0; 
  Kp.back_bkx = 60000.0; 

  Kd.l_arm_usy = 5.0; 
  Kd.l_arm_shx = 5.0; 
  Kd.l_arm_ely = 3.0; 
  Kd.l_arm_elx = 3.0; 
  Kd.l_arm_uwy = 0.1; 
  Kd.l_arm_mwx = 0.1; 
  Kd.l_leg_hpz = 5.0; 
  Kd.l_leg_hpx = 5.0; 
  Kd.l_leg_hpy = 10.0; 
  Kd.l_leg_kny = 10.0; 
  Kd.l_leg_aky = 8.0;
  Kd.l_leg_akx = 2.0;
  Kd.neck_ay = 1.0; 
  Kd.back_bkz = 1.0; 
  Kd.back_bky = 2.0; 
  Kd.back_bkx = 1.0; 
  
elseif strcmpi(mode,'crawling')
  Kp.l_arm_usy = 6000.0; 
  Kp.l_arm_shx = 6000.0; 
  Kp.l_arm_ely = 5000.0; 
  Kp.l_arm_elx = 5000.0; 
  Kp.l_arm_uwy = 2000.0; 
  Kp.l_arm_mwx = 2000.0; 
  Kp.l_leg_hpz = 3000.0; 
  Kp.l_leg_hpx = 3000.0; 
  Kp.l_leg_hpy = 2000.0; 
  Kp.l_leg_kny = 2000.0; 
  Kp.l_leg_aky = 1.0;
  Kp.l_leg_akx = 1.0;
  Kp.neck_ay = 200.0; 
  Kp.back_bkz = 2000.0;  
  Kp.back_bky = 6000.0; 
  Kp.back_bkx = 3000.0; 

  Kd.l_arm_usy = 100.0; 
  Kd.l_arm_shx = 100.0; 
  Kd.l_arm_ely = 100.0; 
  Kd.l_arm_elx = 100.0; 
  Kd.l_arm_uwy = 50.0; 
  Kd.l_arm_mwx = 50.0; 
  Kd.l_leg_hpz = 70.0; 
  Kd.l_leg_hpx = 70.0; 
  Kd.l_leg_hpy = 80.0; 
  Kd.l_leg_kny = 80.0; 
  Kd.l_leg_aky = 1.0;
  Kd.l_leg_akx = 1.0;
  Kd.neck_ay = 20.0; 
  Kd.back_bkz = 50.0; 
  Kd.back_bky = 80.0; 
  Kd.back_bkx = 50.0; 

elseif strcmpi(mode,'gazebo_old')
  Kp.l_arm_usy = 750.0; 
  Kp.l_arm_shx = 1300.0; 
  Kp.l_arm_ely = 800.0; 
  Kp.l_arm_elx = 850.0; 
  Kp.l_arm_uwy = 50.0; 
  Kp.l_arm_mwx = 300.0; 
  Kp.l_leg_hpz = 100.0; 
  Kp.l_leg_hpx = 920.0; 
  Kp.l_leg_hpy = 1550.0; 
  Kp.l_leg_kny = 1850.0; 
  Kp.l_leg_aky = 1900.0;
  Kp.l_leg_akx = 740.0;
  Kp.neck_ay = 50.0; 
  Kp.back_bkz = 800.0;  
  Kp.back_bky = 2100.0; 
  Kp.back_bkx = 800.0; 

  Kd.l_arm_usy = 65.0; 
  Kd.l_arm_shx = 85.0; 
  Kd.l_arm_ely = 55.0; 
  Kd.l_arm_elx = 65.0; 
  Kd.l_arm_uwy = 5.0; 
  Kd.l_arm_mwx = 20.0; 
  Kd.l_leg_hpz = 9.0; 
  Kd.l_leg_hpx = 21.0; 
  Kd.l_leg_hpy = 20.0; 
  Kd.l_leg_kny = 50.0; 
  Kd.l_leg_aky = 57.0;
  Kd.l_leg_akx = 20.0;
  Kd.neck_ay = 2.5; 
  Kd.back_bkz = 60.0; 
  Kd.back_bky = 40.0; 
  Kd.back_bkx = 23.0; 
  
else
  error('unknown mode given. valid modes are: default, pinned, stance_leg, gazebo, crawling.');
end

% copy left gains to right side
Kp.r_arm_usy = Kp.l_arm_usy;
Kp.r_arm_shx = Kp.l_arm_shx;
Kp.r_arm_ely = Kp.l_arm_ely;
Kp.r_arm_elx = Kp.l_arm_elx;
Kp.r_arm_uwy = Kp.l_arm_uwy;
Kp.r_arm_mwx = Kp.l_arm_mwx;
Kp.r_leg_hpz = Kp.l_leg_hpz;
Kp.r_leg_hpx = Kp.l_leg_hpx;
Kp.r_leg_hpy = Kp.l_leg_hpy;
Kp.r_leg_kny = Kp.l_leg_kny;
Kp.r_leg_aky = Kp.l_leg_aky;
Kp.r_leg_akx = Kp.l_leg_akx;

Kd.r_arm_usy = Kd.l_arm_usy;
Kd.r_arm_shx = Kd.l_arm_shx;
Kd.r_arm_ely = Kd.l_arm_ely;
Kd.r_arm_elx = Kd.l_arm_elx;
Kd.r_arm_uwy = Kd.l_arm_uwy;
Kd.r_arm_mwx = Kd.l_arm_mwx;
Kd.r_leg_hpz = Kd.l_leg_hpz;
Kd.r_leg_hpx = Kd.l_leg_hpx;
Kd.r_leg_hpy = Kd.l_leg_hpy;
Kd.r_leg_kny = Kd.l_leg_kny;
Kd.r_leg_aky = Kd.l_leg_aky;
Kd.r_leg_akx = Kd.l_leg_akx;

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end
