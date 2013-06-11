function [qtraj,support_times,supports,S,s1,s2,comtraj,zmptraj] = crawlingPlan(r,x0,options)
%
% @option num_steps will be rounded up to be a multiple of 4
% @option step_speed in m/s
% @option step_height in m
% @option ignore_terrain
% @options direction - 0 for forward, <0 for left, >0 for right 

if nargin<3 || isempty(options), options=struct(); end
if ~isfield(options,'num_steps') options.num_steps = 8; end
if ~isfield(options,'step_speed') options.step_speed = .5; end  
if ~isfield(options,'step_height') options.step_height = .2; end
if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
if ~isfield(options,'direction') options.direction = 0; end
if ~isfield(options,'qnom') options.qnom = x0(1:getNumDOF(r)); end

% always take 4 steps at a time
options.num_steps = 4*ceil(options.num_steps/4);

