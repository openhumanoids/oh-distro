function sys = constructQPFeedbackCombination(r,qp,fc,pd,qt,lfoot_controller,rfoot_controller,pelvis_controller)

	if ~isempty(lfoot_controller) && ~isempty(rfoot_controller)
		% feedback QP controller with atlas
		ins(1).system = 1;
		ins(1).input = 2;
		ins(2).system = 1;
		ins(2).input = 3;
		ins(3).system = 1;
		ins(3).input = 4;
		ins(4).system = 1;
		ins(4).input = 5;
		ins(5).system = 1;
		ins(5).input = 6;
		outs(1).system = 2;
		outs(1).output = 1;
		sys = mimoFeedback(qp,r,[],[],ins,outs);
		clear ins;
	  
	 	% feedback foot contact detector with QP/atlas
	  ins(1).system = 2;
		ins(1).input = 1;
		ins(2).system = 2;
		ins(2).input = 3;
		ins(3).system = 2;
		ins(3).input = 4;
		ins(4).system = 2;
		ins(4).input = 5;
		sys = mimoFeedback(fc,sys,[],[],ins,outs);
	  clear ins;  
	  
		% feedback PD block
		ins(1).system = 1;
		ins(1).input = 1;
		ins(2).system = 2;
		ins(2).input = 2;
		ins(3).system = 2;
		ins(3).input = 3;
		ins(4).system = 2;
		ins(4).input = 4;
		sys = mimoFeedback(pd,sys,[],[],ins,outs);
		clear ins;

		% feedback body motion control blocks
		ins(1).system = 2;
		ins(1).input = 1;
		ins(2).system = 2;
		ins(2).input = 3;
		ins(3).system = 2;
		ins(3).input = 4;
		sys = mimoFeedback(lfoot_controller,sys,[],[],ins,outs);
		clear ins;

		ins(1).system = 2;
		ins(1).input = 1;
		ins(2).system = 2;
		ins(2).input = 3;
		sys = mimoFeedback(rfoot_controller,sys,[],[],ins,outs);
		clear ins;
	else
		% feedback QP controller with atlas
		ins(1).system = 1;
		ins(1).input = 2;
		ins(2).system = 1;
		ins(2).input = 3;
		ins(3).system = 1;
		ins(3).input = 4;
		outs(1).system = 2;
		outs(1).output = 1;
		sys = mimoFeedback(qp,r,[],[],ins,outs);
		clear ins;
	  
	 	% feedback foot contact detector with QP/atlas
	  ins(1).system = 2;
		ins(1).input = 1;
		ins(2).system = 2;
		ins(2).input = 3;
		sys = mimoFeedback(fc,sys,[],[],ins,outs);
	  clear ins;  
	  
		% feedback PD block
		ins(1).system = 1;
		ins(1).input = 1;
		ins(2).system = 2;
		ins(2).input = 2;
		sys = mimoFeedback(pd,sys,[],[],ins,outs);
		clear ins;
	end

	ins(1).system = 2;
	ins(1).input = 1;
	sys = mimoFeedback(pelvis_controller,sys,[],[],ins,outs);
	clear ins;

  sys = mimoFeedback(qt,sys,[],[],[],outs);

end

