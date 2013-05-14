classdef MapHandle < handle
    properties(Hidden=true, SetAccess=private, GetAccess=private)
        mMexFunc;  % which map wrapper mex object to create
        mHandle;   % pointer to the c++ object
    end
    
    methods
        % constructor
        function this = MapHandle(mexFunc)
            if (~exist('mexFunc','var'))
                mexFunc = @mapAPIwrapper;
            end
            this.mMexFunc = mexFunc;
            this.mHandle = mexFunc();
        end

        % destructor
        function delete(this)
            this.mMexFunc(this.mHandle);
        end

        function pts = getPointCloud(this)
            pts = this.mMexFunc(this.mHandle,[]);
        end
        
        function [pts,normals] = getClosest(this,pts)
            [pts,normals] = this.mMexFunc(this.mHandle,pts);
        end
    end
end
