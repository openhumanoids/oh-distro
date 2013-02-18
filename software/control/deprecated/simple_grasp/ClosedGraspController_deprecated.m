classdef ClosedGraspController < DrakeSystem
   % Controller for Atlas opening and closing, input is on/off signal and
   % output is theta_desired for the entire Atlas Robot (for simplicity the
   % only thetas to be position-controlled are the hands not the body
   %This class will be specific to the Atlas robot with hands
   properties
       manip
%        output_frame (Already a prop of DrakeSys)
   end
    methods
        function obj = ClosedGraspController(manip)
            % manip is an instance 
            % HandClosingController(traj)
            % Constructs controller that takes as input the desired
            % trajectory
            typecheck(manip,'TimeSteppingRigidBodyManipulator');
            % Need to figure out a way to parse the manipulator to find out
            % how many states it has for each hand. Will take care of one
            % hand right not but in the future what support for multiframe
            % output to control both hands
            numOutputs=manip.getNumStates/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
%             idx=strmatch('right',manip_with_hand.getStateFrame.coordinates);
%             idx=idx(1:length(idx)/2);
%             hand_coords=manip_with_hand.getStateFrame.coordinates(idx(1):idx(end));
%             frame=CoordinateFrame('grasp',length(idx),'a',hand_coords);

            % For use with a non-floating urdf
            obj=obj@DrakeSystem(0,0,1,numOutputs);  
            coords=manip.getStateFrame.coordinates(1:numOutputs);
            frame_out=CoordinateFrame('q_d',numOutputs,'d',coords); 
            
            % For use with a floating hand urdf, the additional states of
            % the visualizer will be set by altering the pd control in the
            % run function (not in the pd function itself)
%             obj=obj@DrakeSystem(0,0,1,numOutputs-6);  
%             coords=manip.getStateFrame.coordinates(7:numOutputs);
%             frame_out=CoordinateFrame('q_d',numOutputs-6,'a',coords); 
% Noticed that using this gives an error when cascading sys1 and pd in
% runGraspTest

            inp_frame = GraspCmd();
            obj=setInputFrame(obj,inp_frame);
            
            obj=setOutputFrame(obj,frame_out);
            obj.manip=manip;
            
        end
        
%         function getManipulator(hand_manipulator)
            
        
        function y = output(obj,t,x,u)
            % have to incorporate varargin stuff because I am not sure how
            % time comes into the equation.
            if u > .5
                x0 = Point(obj.getOutputFrame);
                
                x0.right_f3_j0=-0.33;
                x0.right_f3_j1=0.46;
                x0.right_f3_j2=1.57;
                
                % x0.right_f2_j0=0;
                x0.right_f2_j1=1.57;
                x0.right_f2_j2=1.57;
                
                % x0.right_f1_j0=0;
                x0.right_f1_j1=1.57;
                x0.right_f1_j2=1.57;
                
                % x0.right_f0_j0=0;
                x0.right_f0_j1=1.57;
                x0.right_f0_j2=1.57;
                
%                 x0 = obj.manip.resolveConstraints(double(x0)); %
%                 Uncommenting this line seems to yield an error that I
%                 don't quite understand, refer to Jan 16 in UROP 2013
%                 google doc for  more details
                y=double(x0);
                
            else
                x0 = Point(obj.getOutputFrame);      
                y=double(x0);
            end
        end
    end
end
            
        