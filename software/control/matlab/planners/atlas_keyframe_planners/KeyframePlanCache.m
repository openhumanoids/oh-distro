classdef KeyframePlanCache
    properties
        num_breaks
        time_2_index_scale
        v_desired
        qdot_desired
        ks
        s_breaks
        qtraj
        isEndPose
        quasiStaticFlag
        num_grasp_transitions
        grasp_transition_breaks
        grasp_transition_states
        s
    end
    
    methods
        function obj = KeyframePlanCache()
            obj.num_breaks = 0;% corresponds to keyframes
            obj.time_2_index_scale = 1;
            obj.v_desired = 0.1;%10cm/s
            obj.qdot_desired = 15*pi/180; %15deg/s
            obj.ks = ActionSequence(); % Plan Boundary Conditions
            obj.s_breaks = [];
            obj.qtraj = [];
            
            % Flag indicates KeyframeAdjustmentEngine to
            % publish an single keyframe endpose instead
            % of a keyframe plan by resolving at time T.
            obj.isEndPose =false;
            obj.quasiStaticFlag = false;
            
            obj.num_grasp_transitions = 0; % corresponds to grasp transitions.
            obj.grasp_transition_breaks = [];
            obj.grasp_transition_states = [];
			      obj.s = [];
        end
    end
end
