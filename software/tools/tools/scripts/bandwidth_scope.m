function read_shaper_usage()
close all
% which bank to compare with [1 is hardest, 5 easist]
bw_run = 2;
% point after the initial startup
% where the bandwidth will be typical
% set to 0.0 to see all the bandwidth
end_of_init_time = 1370036390970730/1E6 % seconds

% bands of rate of BW usage:
if (0==1)
    d.tx.bands =[0.0078125, 0.03125, 0.125, 0.5, 2];
    d.rx.bands =[4,8,16,32,64];
    color_list='ckryg';
else
    d.tx.bands =[0.0078125, 0.03125, 0.125];
    d.rx.bands =[4,8,16];
    color_list='ryg';
end

d=getFile(d, end_of_init_time );
  
current_time_sec= d.sec(end);
tx_total_used = sum(d.tx.data(end,:));
tx_total_budget = d.tx.bands(bw_run)*30*60;
tx_msg=getMsg(tx_total_used,tx_total_budget , current_time_sec, bw_run);
rx_total_used = sum(d.rx.data(end,:));
rx_total_budget = d.rx.bands(bw_run)*30*60;
rx_msg=getMsg(rx_total_used,rx_total_budget , current_time_sec, bw_run);

figure
subplot(2,3,1)
plot( d.sec,   d.rx.data )
legend( d.rx.names,'Location','NorthWest'); 
title('Rx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])

subplot(2,3,2)
hold on
area(d.sec, d.rx.data)
grid on
legend( d.rx.names,'Location','NorthWest'); 
set(gca,'Layer','top')
title ('Cum. Rx [KB]','fontSize',14,'fontWeight','bold') ;% axis([0 inf -inf inf])
for i=1:size(d.rx.bands,2)
    plot(d.sec,  d.sec*d.rx.bands(i) ,['--' color_list(i)],'LineWidth',2)
end
xlabel(['Rx: ' rx_msg],'fontSize',14,'fontWeight','bold')

subplot(2,3,3)
plot( d.sec(2:end),   diff (d.rx.data) )
title('Rate Rx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])

hold on
subplot(2,3,4)
plot( d.sec,   d.tx.data )
legend( d.tx.names,'Location','NorthWest')
title('Tx [KB]','fontSize',14,'fontWeight','bold') ;axis([0 inf -inf inf])

subplot(2,3,5)
hold on
area(d.sec, d.tx.data)
grid on
legend( d.tx.names,'Location','NorthWest'); 
set(gca,'Layer','top')
title ('Cum. Tx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])
for i=1:size(d.rx.bands,2)
    plot(d.sec,  d.sec*d.tx.bands(i) ,['--' color_list(i)],'LineWidth',2)
end
xlabel(['Tx: ' tx_msg],'fontSize',14,'fontWeight','bold')

subplot(2,3,6)
plot( d.sec(2:end),   diff (d.tx.data) )
title('Rate Tx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])


function msg=getMsg(total_used,total_budget , current_time_sec, band_id)
rate = total_used/current_time_sec; %KB/sec
expected_duration = current_time_sec * total_budget  / total_used;
expected_ttl = expected_duration - current_time_sec;
percent_left =  100* (1 - total_used/ total_budget );
expected_ttl_mins = expected_ttl / 60;
msg = [num2str(total_used,'%.1f') ' of ' num2str(total_budget,'%.1f') 'KB | Rate ' num2str(rate,'%10.2f') 'KB/sec | ' num2str(percent_left,'%2.2f') '% left | TTL: ' num2str(expected_ttl_mins,'%2.2f') 'mins [band=' num2str(band_id) ']'];

function d=getFile(d,end_of_init_time )
path = '~/drc/software/config/'
files = dir([path 'drc-network-shaper-data-usage-base-*.csv']);
filename = files(end).name

%path = '/home/mfallon/drc/software/tools/tools/scripts/';
%filename ='drc-network-shaper-data-usage-example.csv'; 
%filename = 'drc-network-shaper-data-usage-example-newer.csv';

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
end

% scale from bytes to kbytes
scale = 1024

d.rx.names = names( direction==0 );
d.rx.data = data(: , (direction ==0) );
d.rx.names  = d.rx.names (  d.rx.data(end,:) ~=0 );
d.rx.data  = d.rx.data (:,  d.rx.data(end,:) ~=0 )/ scale;
d.tx.names = names( direction==1 );
d.tx.data = data(:, (direction ==1) );
d.tx.names  = d.tx.names (  d.tx.data(end,:) ~=0 );
d.tx.data  = d.tx.data (:,  d.tx.data(end,:) ~=0 )/ scale;

% remove the burp at the start of our operation:
start_idx = find ( d.sec  >= end_of_init_time , 1 , 'first');
idx_keep = [start_idx : size(d.sec,1)];
tx_data_blurp = d.tx.data(start_idx, :);
d.tx.data = d.tx.data(idx_keep, :) - repmat ( tx_data_blurp,  size(d.tx.data(idx_keep, :),1 ),1 ) ;
rx.data_blurp = d.rx.data(start_idx, :);
d.rx.data = d.rx.data(idx_keep, :) - repmat ( rx.data_blurp,  size(d.rx.data(idx_keep, :),1 ),1 ) ;
d.sec = d.sec(idx_keep);

d.sec= d.sec - end_of_init_time;