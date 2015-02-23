function labels = find_connected_regions(in,max_dval,min_size)

w = size(in,2);
h = size(in,1);
labels = zeros(size(in));
equivalences = zeros(w*h,1);

cur_label = 0;
for i = 2:h
    for j = 2:w
        label_above = collapse(labels(i-1,j),equivalences);
        label_left = collapse(labels(i,j-1),equivalences);
        val = in(i,j);
        val_above = in(i-1,j);
        val_left = in(i,j-1);
        if (label_above>0) && (abs(val-val_above)<max_dval)
            labels(i,j) = label_above;
            if (label_left>0) && (label_above ~= label_left) && (abs(val-val_left)<max_dval)
                equivalences(label_left) = label_above;
            end
        else
            if (label_left>0) && (abs(val-val_left)<max_dval)
                labels(i,j) = label_left;
            else
                cur_label = cur_label+1;
                labels(i,j) = cur_label;
                equivalences(cur_label) = cur_label;
            end
        end
    end
end

% collapse equivalences
for i = 1:cur_label
    equivalences(i) = collapse(i,equivalences);
end

% remap
labels(labels>0) = equivalences(labels(labels>0));

% count
u = unique(labels);
u(u==0) = [];
h = hist(labels(labels>0),u);
labels(ismember(labels,u(h<min_size))) = 0;

% remap again
u = unique(labels(labels>0));
mapper = zeros(max(u),1);
mapper(u) = 1:numel(u);
labels(labels>0) = mapper(labels(labels>0));


function out = collapse(in, equivalences)
out = in;
if (out == 0)
    return
end
while (equivalences(out) ~= out)
    out = equivalences(out);
end

