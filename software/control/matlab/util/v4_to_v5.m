function q_5 = v4_to_v5(r4,r5,q_4)
	names_4 = r4.getStateFrame.coordinates(1:34);
  q_5 = zeros(36,1);
  for j = 1:numel(names_4)
    name = names_4{j};
    q_5(r5.findPositionIndices(name)) = q_4(r4.findPositionIndices(name));
  end
end