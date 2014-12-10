function [matches, scores] = match_sift_features(x,y)

x = double(x');
y = double(y');
x2 = sum(x.^2,2);
y2 = sum(y.^2,2);

scores = zeros(size(x,1),size(y,1));

if (size(x,1) < size(y,1))
    for i = 1:size(x,1)
        scores(i,:) = y*x(i,:)';
    end
else
    for i = 1:size(y,1)
        scores(:,i) = x*y(i,:)';
    end
end    

for i = 1:size(x,1)
    scores(i,:) = -2*scores(i,:) + x2(i);
end
for i = 1:size(y,1)
    scores(:,i) = scores(:,i) + y2(i);
end

[minval1,minidx1] = min(scores,[],2);
ind = sub2ind(size(scores), 1:size(scores,1),minidx1');
scores(ind) = 1e10;
[minval2,minidx2] = min(scores,[],2);

ind = 1.5*minval1 < minval2;
matches = [find(ind),minidx1(ind)]';
scores = minval1(ind);
