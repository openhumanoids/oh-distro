function [ imu, noiseState ] = addIMUNoise( imu_, noiseState_ )
%ADDIMUNOISE Summary of this function goes here
%   Detailed explanation goes here

  % Initialize the noiseState data structure for use in this function
  if (length(noiseState_)==0)
      noiseState.something = 0;
  end
  
  % Just
  imu = imu_;
  noiseState = noiseState_;
  

end

