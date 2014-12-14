f = zeros(20,30,3);

for i = 1:20
    for j=1:30
        for k=1:3
            if k==2
                f(i,j,k) = 1.0;
%             elseif(k==2)
%                  f(i,j,k) = 0.5;
             end
        end
    end
end

imshow(f);
hsv = rgb2hsv(f);
% imshow(hsv);

% imshow(f);