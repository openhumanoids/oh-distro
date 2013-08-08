close all
figure

x1= imread('../test.png');
subplot(2,2,1)
imagesc(x1)
colorbar

x= imread('../test_gray.png');
subplot(2,2,2)
imagesc(x)
colorbar

x= imread('../test_morph_gray.png');
subplot(2,2,3)
imagesc(x)
colorbar

x4= imread('../test_morph_gray_threshold.png');
subplot(2,2,4)
imagesc(x4)
colorbar

figure
x= imread('../test_out.png');
subplot(1,2,2)
imagesc(x)
colorbar

subplot(1,2,1)
imagesc(x1)
colorbar

% 130 358 945