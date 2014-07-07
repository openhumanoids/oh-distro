classdef HeightMapHandle < handle
    properties(Hidden=true, SetAccess=private, GetAccess=private)
        mMexFunc;  % which map wrapper mex object to create
        mHandle;   % pointer to the c++ object
    end
    
    methods(Access=private)
        function out = logicalToString(~,val)
            if (val)
                out = 'true';
            else
                out = 'false';
            end
        end
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
            this.mMexFunc('property',this.mHandle,'fillmissing',this.logicalToString(val));
        end
        
        function overrideHeights(this,val)
            this.mMexFunc('property',this.mHandle,'overrideheights',this.logicalToString(val));
        end
        
        function setFillPlane(this,plane)
            this.mMexFunc('fillplane',this.mHandle,plane);
        end
        
        function setUseFootPose(this,val)
            this.mMexFunc('property',this.mHandle,'usefootpose',this.logicalToString(val));
        end            
        
        function [heights,normals] = getTerrain(this,xy)
            [heights,normals] = this.mMexFunc('terrain',this.mHandle,xy);
        end
        
        function setNormalRadius(this,radius)
            this.mMexFunc('property',this.mHandle,'normalradius',sprintf('%f',radius));
        end

        function setNormalMethod(this,method)
            if (strcmpi(method,'override'))
                this.mMexFunc('property',this.mHandle,'normalmethod','0');
            elseif (strcmpi(method,'leastsquares'))
                this.mMexFunc('property',this.mHandle,'normalmethod','2');
            elseif (strcmpi(method,'robust'))
                this.mMexFunc('property',this.mHandle,'normalmethod','3');
            elseif (strcmpi(method,'ransac'))
                this.mMexFunc('property',this.mHandle,'normalmethod','4');
            else
                error('invalid normal method');
            end
        end

        function ptr = getPointerForMex(this)
            ptr = this.mMexFunc('pointer',this.mHandle);
        end
        
        function [heights,xform] = getHeightData(this)
            [heights,xform] = this.mMexFunc('heightdata',this.mHandle);
            xform = inv(xform);
            [x,y] = meshgrid(1:size(heights,2),1:size(heights,1));
            pts = [x(:),y(:),heights(:)];
            pts = [pts,ones(size(pts,1),1)]*xform(3:4,:)';
            heights = reshape(pts(:,1)./pts(:,2),size(x));
            xform(3,:) = [0,0,1,0];
        end
    end
end
