function ind = bdiInd(floating)
%bdiInd returns index array that maps atlas state indices to BDI's internal ordering
if nargin < 1
  floating = true;
else
  isa(floating,'logical');
end

%ind = 6*floating+[1 2 3 17 18 19 20 21 5 6 7 8 9 10 22 23 24 25 26 27 11 12 13 14 15 16 28 4]';
ind = 6*floating+[1 2 3 28 9 10 11 12 13 14 21 22 23 24 25 26 4 5 6 7 8 15 16 17 18 19 20 27]';

end

