classdef HeightMapHandle < handle
    properties(Hidden=true, SetAccess=private, GetAccess=private)
        mMexFunc;  % which map wrapper mex object to create
        mHandle;   % pointer to the c++ object
    end
    
    methods
        % constructor
        function this = HeightMapHandle(mexFunc,privateChannel)
            if (~exist('mexFunc','var'))
                mexFunc = @HeightMapWrapper;
            end
            if (~exist('privateChannel','var'))
                privateChannel = 'false';
            end
            this.mMexFunc = mexFunc;
            this.mHandle = mexFunc('create','privatechannel',privateChannel);
        end

        % destructor
        function delete(this)
            this.mMexFunc('destroy',this.mHandle);
        end
        
        function this = setFillMissing(this,val)
            if (val)
                this.mMexFunc('property',this.mHandle,'fill','true');
            else
                this.mMexFunc('property',this.mHandle,'fill','false');
            end
        end
        
        function pts = getPointCloud(this)
            pts = this.mMexFunc('pointcloud',this.mHandle);
        end
        
        function [pts,normals] = getClosest(this,pts)
            [pts,normals] = this.mMexFunc('closest',this.mHandle,pts);
        end
    end
end
