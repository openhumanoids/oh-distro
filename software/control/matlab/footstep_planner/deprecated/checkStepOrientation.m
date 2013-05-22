function c = checkStepOrientation(biped, p)

c = zeros(size(p, 2) - 2);
for j = 1:(size(p, 2) - 2)
  theta = atan2(p(2, j+2) - p(2, j), p(1, j+2) - p(1, j));
  c(j) = abs(theta - p(6, j)) - 0.05;
end

c = reshape(c, [], 1);
end

