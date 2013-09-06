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
    end
    
    methods
        function obj = KeyframePlanCache()
            obj.num_breaks = 0;
            obj.time_2_index_scale = 1;
            obj.v_desired = 0.1;%10cm/s
            obj.qdot_desired = 2*pi/180; %2deg/s
            obj.ks = ActionSequence(); % Plan Boundary Conditions
            obj.s_breaks = [];
            obj.qtraj = [];
            
            % Flag indicates KeyframeAdjustmentEngine to
            % publish an single keyframe endpose instead
            % of a keyframe plan by resolving at time T.
            obj.isEndPose =false;
            obj.quasiStaticFlag = false;
        end
    end
end
