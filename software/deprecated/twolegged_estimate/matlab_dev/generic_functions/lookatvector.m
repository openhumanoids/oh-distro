function [y] = lookatvector(Out,start,stop,var_str)

[r,c] = size(eval(['Out{1}.' var_str]));

if (or(r==1,c==1))
	
	if (r==1)
		y = zeros(stop-start,c);
	else
		y = zeros(stop-start,r);
	end
	
	for n=1:(stop-(start-1))
		value = eval(['Out{' num2str(n+(start-1)) '}.' var_str]);
		if (r==1)
			y(n,:) = value;
		else
			y(n,:) = value';
		end
	end
end
