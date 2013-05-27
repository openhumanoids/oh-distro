function read_shaper_usage()
% point after the initial startup
% where the bandwidth will be typical
% set to 0.0 to see all the bandwidth
end_of_init_time = 1.0

close all
d=getFile( end_of_init_time );

if (1==0)
    bands_rx =[256,128,64,32]
    bands_tx =[4,1,0.25,0.0625]
    color_list='kryg'
else
    bands_rx =[128,64,32]
    bands_tx =[1,0.25,0.0625]
    color_list='ryg'   
end

figure
subplot(2,3,1)
plot( d.sec,   d.data_rx )
legend( d.names_rx,'Location','NorthWest'); 
title('Rx [KB]'); axis([0 inf -inf inf])


subplot(2,3,2)
hold on
area(d.sec, d.data_rx)
grid on
legend( d.names_rx)
set(gca,'Layer','top')
title ('Cum. Rx [KB]') ; axis([0 inf -inf inf])
for i=1:size(bands_rx,2)
    plot(d.sec,  cumsum(d.sec,2)*bands_rx(i) ,['--' color_list(i)],'LineWidth',2)
end

subplot(2,3,3)
plot( d.sec(2:end),   diff (d.data_rx) )
title('Rate Rx [KB]'); axis([0 inf -inf inf])




hold on
subplot(2,3,4)
plot( d.sec,   d.data_tx )
legend( d.names_tx,'Location','NorthWest')
title('Tx [KB]') ;axis([0 inf -inf inf])

subplot(2,3,5)
hold on
area(d.sec, d.data_tx)
grid on
legend( d.names_tx)
set(gca,'Layer','top')
title ('Cum. Tx [KB]'); axis([0 inf -inf inf])
for i=1:size(bands_rx,2)
    plot(d.sec,  cumsum(d.sec,2)*bands_tx(i) ,['--' color_list(i)],'LineWidth',2)
end

subplot(2,3,6)
plot( d.sec(2:end),   diff (d.data_tx) )
title('Rate Tx [KB]'); axis([0 inf -inf inf])




function d=getFile(end_of_init_time )
path = '~/drc/software/config/'
files = dir([path 'drc-network-shaper-data-usage-base-*']);
filename = files(end).name 


fid=fopen([path filename]);
tline = fgetl(fid);
names = regexp(tline,',','split');
names = names(2:end);
data= dlmread([path filename],',',1,0);
d.sec = data(:,1)*1E-6;
data = data(:,[2:end]);






for i=1:size(names,2)
   direction(i) = str2num(names{i}(end));
   temp = names{i}(1:end-1);
   temp = strrep(temp, '_', '-');
   names{i} = temp;
   
   %d.names(i)(1:end-1)
end

% scale from bytes to kbytes
scale = 1024

d.names_rx = names( direction==0 );
d.data_rx = data(: , (direction ==0) );
d.names_rx  = d.names_rx (  d.data_rx(end,:) ~=0 );
d.data_rx  = d.data_rx (:,  d.data_rx(end,:) ~=0 )/ scale;

d.names_tx = names( direction==1 );
d.data_tx = data(:, (direction ==1) );
d.names_tx  = d.names_tx (  d.data_tx(end,:) ~=0 );
d.data_tx  = d.data_tx (:,  d.data_tx(end,:) ~=0 )/ scale;



% remove the burp at the start of our operation:
start_idx = find ( d.sec  >= end_of_init_time , 1 , 'first');
idx_keep = [start_idx : size(d.sec,1)];
data_tx_blurp = d.data_tx(start_idx, :);
d.data_tx = d.data_tx(idx_keep, :) - repmat ( data_tx_blurp,  size(d.data_tx(idx_keep, :),1 ),1 ) ;
data_rx_blurp = d.data_rx(start_idx, :);
d.data_rx = d.data_rx(idx_keep, :) - repmat ( data_rx_blurp,  size(d.data_rx(idx_keep, :),1 ),1 ) ;
d.sec = d.sec(idx_keep);
