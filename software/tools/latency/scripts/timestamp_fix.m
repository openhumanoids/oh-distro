close all; clear all
%d = importdata('~/Desktop/timestamp_pre_fix.txt')
d = importdata('~/Desktop/timestamp_post_fix.txt')

nz_bdi = d(:,2) ~=0
nz_crl = d(:,3) ~=0
min_t = min(d(:,1))

figure;
subplot(3,1,1)

plot( 1E-6*(d(nz_bdi,1) - min_t), d(nz_bdi,2), '.r')
subplot(3,1,2)
plot( 1E-6*(d(nz_crl,1) - min_t), d(nz_crl,3), '.b')

subplot(3,1,3)
hold on
plot( 1E-6*(d(nz_bdi,1) - min_t), d(nz_bdi,2), '.r')
plot( 1E-6*(d(nz_crl,1) - min_t), d(nz_crl,3), '.b')
