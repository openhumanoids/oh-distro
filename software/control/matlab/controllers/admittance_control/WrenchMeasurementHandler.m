classdef WrenchMeasurementHandler < handle
    
    properties
        r
        hardware_mode % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
        r_hand_body
        l_hand_body
        r_foot_body
        l_foot_body
        l_issandia
        r_issandia
        
        T_palm_hand_l
        T_palm_hand_r
        T_hand_palm_l
        T_hand_palm_r
        T_hand_palm_l_sandia
        T_hand_palm_r_sandia
        T_hand_palm_l_irobot
        T_hand_palm_r_irobot
        
        hand_mass_l
        hand_mass_r
        hand_mass_l_sandia
        hand_mass_r_sandia
        hand_mass_l_irobot
        hand_mass_r_irobot
        hand_com_offset_l_sandia
        hand_com_offset_r_sandia
        hand_com_offset_l_irobot
        hand_com_offset_r_irobot
        
        hand_sensor_offset_l
        hand_sensor_offset_r
        hand_com_offset_l
        hand_com_offset_r
        
        gain_l
        gain_r
        hand_sensor_wrenchoffset_l
        hand_sensor_wrenchoffset_r
        
        compensate_for_hand_mass
    end
    
    methods
        function obj = WrenchMeasurementHandler(r,hardware_mode)
            obj.r = r;
            obj.hardware_mode=hardware_mode;
            obj.r_hand_body = findLinkInd(obj.r,'r_hand');
            obj.l_hand_body = findLinkInd(obj.r,'l_hand');
            obj.r_foot_body = obj.r.findLinkInd('r_foot');
            obj.l_foot_body = obj.r.findLinkInd('l_foot');
            
            % Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            ft_sensor_offset = 0.045; % approx 1.8 inches
          
            obj.T_hand_palm_l_sandia = obj.HT([0;0.1+ft_sensor_offset;0],0,0,1.57079);
            obj.T_hand_palm_r_sandia = obj.HT([0;-(0.1+ft_sensor_offset);0],0,0,-1.57079);
            obj.T_hand_palm_l_irobot = obj.HT([0;0.05+ft_sensor_offset;0],1.57079,0,3.14159);
            obj.T_hand_palm_r_irobot = obj.HT([0;-(0.05+ft_sensor_offset);0],1.57079,0,0);
            
            % The wrist force/torque sensor reference point is located in the hand frame:
            % offset = (0 -0.1245 -0.0112) right hand
            % offset = (0  0.1245 -0.0112) left hand
            obj.hand_sensor_offset_l = [0;0.1245;-0.0112];
            obj.hand_sensor_offset_r = [0;-0.1245;-0.0112];

            % hand com on l/r_hand frame
            obj.hand_com_offset_l_sandia = [0.0087;0.2053;0.0471];
            obj.hand_com_offset_r_sandia  = [0.0087;-0.2053;0.0471];
            obj.hand_com_offset_l_irobot = [0.00;0.1651;0.1615];
            obj.hand_com_offset_r_irobot = [0.00;-0.1651;0.1615];

            % also compensate for sensor offset here as a virtual mass?
            obj.hand_mass_r_sandia = 3.131;
            obj.hand_mass_l_sandia = obj.hand_mass_r_sandia;
            obj.hand_mass_r_irobot = 0.9;
            obj.hand_mass_l_irobot = obj.hand_mass_r_irobot;
         
            
            obj.gain_l=1;
            obj.hand_sensor_wrenchoffset_l = zeros(6,1);  
            obj.gain_r=1;           
            obj.hand_sensor_wrenchoffset_r = zeros(6,1); 
            %obj.gain_l=0.5;
            % sensor reading without the hand
            %obj.hand_sensor_wrenchoffset_l = [6.4055;18.14;10.6916;0.45;-0.411;-0.90];
            obj.gain_l=1;
            % sensor reading with the hand (screw tensioning may affect the ft sensor output)
            obj.hand_sensor_wrenchoffset_l = [6.2816;74.61;7.11;0.85;-0.644;-2.25];
            obj.compensate_for_hand_mass = false;
            if(hardware_mode==1)% sim mode
                obj.compensate_for_hand_mass = true;
                obj.gain_l=1;
                obj.gain_r=1;
                obj.hand_sensor_wrenchoffset_l = zeros(6,1);
                obj.hand_sensor_wrenchoffset_r = zeros(6,1);
                % in gazebo, ft sensor is defined in l/r_hand and includes the
                % hand link weight as gazebo welds fixed joint links together.
                obj.hand_sensor_offset_l = 0*[0;0.1245;-0.0112];
                obj.hand_sensor_offset_r = 0*[0;-0.1245;-0.0112];   
                % com is defined including l_hand mass in sim.
                obj.hand_com_offset_l_sandia = [0.0048;0.1140;0.0021];
                obj.hand_com_offset_r_sandia = [0.0048;-0.1140;0.0021];
                r_hand_link_mass=r.getBody(r.findLinkInd('r_hand')).mass;
                l_hand_link_mass=r.getBody(r.findLinkInd('l_hand')).mass;
                obj.hand_mass_r_sandia = 3.131+r_hand_link_mass;
                obj.hand_mass_l_sandia = 3.131+l_hand_link_mass;
                obj.hand_mass_r_irobot = 0.9+r_hand_link_mass;
                obj.hand_mass_l_irobot = 0.9+l_hand_link_mass;
            end
            
            obj.setHandType(true,true); % set sandia hands as default
            
        end
        
        function setHandType(obj,l_issandia,r_issandia)
            obj.l_issandia = l_issandia;
            obj.r_issandia = r_issandia;
            
            if(obj.l_issandia)
                obj.T_hand_palm_l = obj.T_hand_palm_l_sandia;
                obj.hand_mass_l = obj.hand_mass_l_sandia;
                obj.hand_com_offset_l=obj.hand_com_offset_l_sandia;
            else
                obj.T_hand_palm_l = obj.T_hand_palm_l_irobot;
                obj.hand_mass_l = obj.hand_mass_l_irobot;
                obj.hand_com_offset_l=obj.hand_com_offset_l_irobot;
            end
            
            if(obj.l_issandia)
                obj.T_hand_palm_r = obj.T_hand_palm_r_sandia;
                obj.hand_mass_r = obj.hand_mass_r_sandia;
                obj.hand_com_offset_r=obj.hand_com_offset_r_sandia;
            else
                obj.T_hand_palm_r = obj.T_hand_palm_r_irobot;
                obj.hand_mass_r = obj.hand_mass_r_irobot;
                obj.hand_com_offset_l=obj.hand_com_offset_r_irobot;
            end
            
            obj.T_palm_hand_l = obj.inv_HT(obj.T_hand_palm_l);
            obj.T_palm_hand_r = obj.inv_HT(obj.T_hand_palm_r);
        end
        %-----------------------------------------------------------------------------------------------------------------
        function setHardwareMode(obj,mode)
            obj.hardware_mode = mode;
        end
        %----------obj-------------------------------------------------------------------------------------------------------
        function val=isSimMode(obj)
            val = (obj.hardware_mode == 1);
        end
        %-----------------------------------------------------------------------------------------------------------------
        function val=isBDIManipMode(obj)
            val = (obj.hardware_mode == 2);
        end
        %-----------------------------------------------------------------------------------------------------------------
        function val=isBDIUserMode(obj)
            val = (obj.hardware_mode == 3);
        end
        %-----------------------------------------------------------------------------------------------------------------
        function  w_hand_frame = compensate_hand_wrenches_for_hand_mass_and_sensor_offset(obj,x0,w_sensor_frame)
            % performs three things in wrench space.
            % 1) convert sensor frame wrench measurements to world frame
            % 2) compensate for hand mass and calibration offsets defined
            % in world orientation.
            % 3) convert wrench in world frame back to hand frame.
            
            q0 = x0(1:getNumDOF(obj.r));
            %T_world_body = obj.HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get hand positions
            kinsol = doKinematics(obj.r,q0);
            
            % ---------------------------------------------------------
            % 1) convert sensor frame wrench measurements to world frame
            % ---------------------------------------------------------
            r_sensor_pose = forwardKin(obj.r,kinsol,obj.r_hand_body,obj.hand_sensor_offset_r,1);
            l_sensor_pose = forwardKin(obj.r,kinsol,obj.l_hand_body,obj.hand_sensor_offset_l,1);
            WT_r = obj.wrench_transform(r_sensor_pose(1:3),r_sensor_pose(4),r_sensor_pose(5),r_sensor_pose(6));
            WT_l = obj.wrench_transform(l_sensor_pose(1:3),l_sensor_pose(4),l_sensor_pose(5),l_sensor_pose(6));
            w_sensor_frame.lh=obj.gain_l.*(w_sensor_frame.lh(:)-obj.hand_sensor_wrenchoffset_l);
            w_sensor_frame.rh=obj.gain_r.*(w_sensor_frame.rh(:)-obj.hand_sensor_wrenchoffset_r);
            w_world_frame.lh = WT_l*w_sensor_frame.lh(:);
            w_world_frame.rh = WT_r*w_sensor_frame.rh(:);
            
            
            % ---------------------------------------------------------
            % 2)  compensate for hand mass and calibration offsets
            % ---------------------------------------------------------
            if(obj.compensate_for_hand_mass)
                % define gravity wrench due to hand mass to world frame
                r_hand_com_pose = forwardKin(obj.r,kinsol,obj.r_hand_body,obj.hand_com_offset_r,1);
                l_hand_com_pose = forwardKin(obj.r,kinsol,obj.l_hand_body,obj.hand_com_offset_l,1);
                g= -9.81; %m/s2
                w_world_handmass_r = zeros(6,1);
                w_world_handmass_r(3)= obj.hand_mass_r*g;
                w_world_handmass_l = zeros(6,1);
                w_world_handmass_l(3)= obj.hand_mass_l*g;

               % ignoring hand posture, assuming fixed joints.
                r_hand_com_position = r_hand_com_pose(1:3);
                l_hand_com_position = l_hand_com_pose(1:3);

                % shift wrench to origin
                WT_r = obj.wrench_transform(r_hand_com_position,0,0,0);
                WT_l = obj.wrench_transform(l_hand_com_position,0,0,0);
                w_world_frame.rh =  w_world_frame.rh - WT_r* w_world_handmass_r(:);
                w_world_frame.lh =  w_world_frame.lh - WT_l* w_world_handmass_l(:);
            end
            
            % ---------------------------------------------------------
            % 3) convert wrench in world frame back to hand frame.
            % ---------------------------------------------------------
            r_hand_pose = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],1);
            l_hand_pose = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],1);                        
            T_hand_world_r=obj.inv_HT(obj.HT(r_hand_pose(1:3),r_hand_pose(4),r_hand_pose(5),r_hand_pose(6)));
            T_hand_world_l=obj.inv_HT(obj.HT(l_hand_pose(1:3),l_hand_pose(4),l_hand_pose(5),l_hand_pose(6)));
            WT_r = obj.homogeneous_to_wrench_transform(T_hand_world_r);
            WT_l = obj.homogeneous_to_wrench_transform(T_hand_world_l);
            w_hand_frame.rh = WT_r*w_world_frame.rh(:);
            w_hand_frame.lh = WT_l*w_world_frame.lh(:);
            
        end
    end
    methods(Static)
        %-----------------------------------------------------------------------------------------------------------------
        function WT= wrench_transform(p,roll,pitch,yaw)
            % changing frame of reference for wrenches.
            % world_frame_wrench(6x1) = [R_world_body,0; p_world_body X R_world_body, R_world_body](6x6) * body_frame_wrench (6x1);
            % where X is cross product. p_world_body X  can be represented via a
            % skew symmetric matrix.
            WT = zeros(6);
            M = rotz(yaw)*roty(pitch)*rotx(roll);
            S=WrenchMeasurementHandler.XProdSkew3d(p);
            WT(1:3,1:3) = M;
            WT(4:6,1:3) = S*M;
            WT(4:6,4:6) = M;
        end
        %-----------------------------------------------------------------------------------------------------------------
        function WT= homogeneous_to_wrench_transform(HT)
            
            WT = zeros(6);
            M = HT(1:3,1:3);
            S=WrenchMeasurementHandler.XProdSkew3d(HT(1:3,4));
            WT(1:3,1:3) = M;
            
            WT(4:6,1:3) = S*M;
            WT(4:6,4:6) = M;
        end
        %-----------------------------------------------------------------------------------------------------------------
        function T= HT(p,roll,pitch,yaw)
            T = zeros(4);
            M = rotz(yaw)*roty(pitch)*rotx(roll);
            
            T(1:3,1:3) = M;
            T(1:4,4) = [p(:); 1];
        end
        %-----------------------------------------------------------------------------------------------------------------
        function T_out=inv_HT(T)
            M = T(1:3,1:3);
            p = T(1:3,4);
            T_out = zeros(4);
            T_out(1:3,1:3) = M';
            p= -M'*p;
            T_out(1:4,4) = [p(:); 1];
        end
        %-----------------------------------------------------------------------------------------------------------------
        function [S]=XProdSkew3d(v)
            v = v(:); % v must be 3x1
            S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
        %-----------------------------------------------------------------------------------------------------------------
    end
end
