classdef SchmittTrigger < handle
  properties
    input_low_to_high_threshold = 0.75;
    input_high_to_low_threshold = 0.25;
    output_high = 1;
    output_low = 0;
    state = 0;
  end

  methods
    function obj = SchmittTrigger(output_low, output_high, input_high_to_low_threshold, input_low_to_high_threshold)
      obj.output_low = output_low;
      obj.output_high = output_high;
      obj.input_high_to_low_threshold = input_high_to_low_threshold;
      obj.input_low_to_high_threshold = input_low_to_high_threshold;
    end

    function output = update(obj, u)
      if u >= obj.input_low_to_high_threshold
        obj.state = obj.output_high;
      elseif u <= obj.input_high_to_low_threshold
        obj.state = obj.output_low;
      end
      output = obj.state;
    end
  end
end




