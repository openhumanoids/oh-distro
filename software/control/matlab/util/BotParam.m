classdef BotParam < handle
    properties(Access=private)
        rootStr = '';
    end
    methods
        function this = BotParam(str)
            if (nargin < 1)
                str = '';
            end
            this.rootStr = str;
        end
        
        function out = get(this,key)
            if (numel(this.rootStr) > 0)
                key = sprintf('%s.%s',this.rootStr,key);
            end
            if (~BotParamClient('haskey', key))
                error('key %s does not exist', key);
            end
            try
                out = BotParamClient('getnum',key);
            catch ex
                try
                    out = BotParamClient('getbool',key);
                catch ex
                    try
                        out = BotParamClient('getstr',key);
                    catch ex
                        error('cannot parse key %s', key);
                    end
                end
            end
        end
        
        function set(this,key,val)
            if (numel(this.rootStr) > 0)
                key = sprintf('%s.%s',this.rootStr,key);
            end
            if (ischar(val) || iscell(val))
                BotParamClient('setstr',key,val);
            elseif (isreal(val))
                BotParamClient('setnum',key,val);
            end
        end
        
        %         function out = subsref(this,dat)
        %             str = this.rootStr;
        %             for i = 1:numel(dat)
        %                 if (~strcmp(dat(i).type,'.'))
        %                     error('invalid syntax');
        %                 end
        %                 if (numel(str) == 0)
        %                     str = dat(i).subs;
        %                 else
        %                     str = sprintf('%s.%s', str, dat(i).subs);
        %                 end
        %             end
        %             out = BotParam(str);
        %         end
    end
end
