function out = decimate_points(in,res)

min_pt = min(in,[],1);
pts_int = round((in-repmat(min_pt,[size(in,1),1]))/res)+1;
sz = max(pts_int,[],1);
pts_ind = sub2ind(sz,pts_int(:,1),pts_int(:,2),pts_int(:,3));
[sorted_ind,sort_idx] = sort(pts_ind);
in = in(sort_idx,:);
d = diff(sorted_ind);
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];
out = zeros(numel(starts),3);
for i = 1:numel(starts)
    idx = starts(i):ends(i);
    pts = in(idx,:);
    out(i,:) = mean(pts,1);
end
