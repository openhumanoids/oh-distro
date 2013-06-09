function bandwidth_scope()

close all
% which bank to compare with [1 is hardest, 5 easist]
p.scale = 1024
p.bw_run =1 ;% 5 easy 1 hard

% bands of rate of BW usage:
if (0==1)
    d.tx.bands =[0.0078125, 0.03125, 0.125, 0.5, 2];
    d.rx.bands =[4,8,16,32,64];
    p.color_list='rygck';
else
    d.tx.bands =[0.0078125, 0.03125, 0.125];
    d.rx.bands =[4,8,16];
    p.color_list='ryg';
end


d=getFile(d ,p);
plotShaper(d,p);

%vrc=[]
%vrc=getVRCScoreFile( vrc,p,d )
%plotVRCScore(vrc,p)


function plotVRCScore(d,p)
figure
subplot(2,2,1)
plot( d.sec,   d.rx.data )
title('Rx Remaining [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf 0 inf])


subplot(2,2,3)
plot( d.sec,   d.tx.data )
title('Tx Remaining [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf 0 inf])

subplot(2,2,2)
plot( d.sec(2:end),   -diff (d.rx.data) )
title('Rate Rx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])

subplot(2,2,4)
plot( d.sec(2:end),   -diff (d.tx.data) )
title('Rate Tx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])


function plotShaper(d,p)
current_time_sec= d.sec(end);
tx_total_used = sum(d.tx.data(end,:));
tx_total_budget = d.tx.bands(p.bw_run)*30*60;
tx_msg=getMsg(tx_total_used,tx_total_budget , current_time_sec, p.bw_run);
rx_total_used = sum(d.rx.data(end,:));
rx_total_budget = d.rx.bands(p.bw_run)*30*60;
rx_msg=getMsg(rx_total_used,rx_total_budget , current_time_sec, p.bw_run);

figure
subplot(2,3,1)

if ( size(d.rx.data,2) ==0)
    disp('No RX Data Yet')
    subplot(2,3,2)
    xlabel('No Rx Data Yet','fontSize',14,'fontWeight','bold')    
else  
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
      plot(d.sec,  d.sec*d.rx.bands(i) ,['--' p.color_list(i)],'LineWidth',2)
    end
    xlabel(['Rx: ' rx_msg],'fontSize',14,'fontWeight','bold')
    
    subplot(2,3,3)
    hold on
    rx_rate_overall = sum(diff (d.rx.data)');
    plot( d.sec(2:end),   rx_rate_overall , 'r', 'LineWidth', 2 );
    plot( d.sec(2:end),   diff (d.rx.data) )
    title('Rate Rx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])

end

if ( size(d.tx.data,2) ==0)
    disp('No TX Data Yet')
    subplot(2,3,5)
    xlabel('No Tx Data Yet','fontSize',14,'fontWeight','bold')    
else
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
        plot(d.sec,  d.sec*d.tx.bands(i) ,['--' p.color_list(i)],'LineWidth',2)
    end
    xlabel(['Tx: ' tx_msg],'fontSize',14,'fontWeight','bold')
    
    subplot(2,3,6)
    hold on
    tx_rate_overall = sum(diff (d.tx.data)',1);
    plot( d.sec(2:end),   tx_rate_overall ,'r', 'LineWidth', 2 );
    plot( d.sec(2:end),   diff (d.tx.data) )
    title('Rate Tx [KB]','fontSize',14,'fontWeight','bold'); axis([0 inf -inf inf])
end

function msg=getMsg(total_used,total_budget , current_time_sec, band_id)
rate = total_used/current_time_sec; %KB/sec
expected_duration = current_time_sec * total_budget  / total_used;
expected_ttl = expected_duration - current_time_sec;
percent_left =  100* (1 - total_used/ total_budget );
expected_ttl_mins = expected_ttl / 60;
msg = [num2str(total_used,'%.1f') ' of ' num2str(total_budget,'%.1f') 'KB | Rate ' num2str(rate,'%10.2f') 'KB/sec | ' num2str(percent_left,'%2.2f') '% left | TTL: ' num2str(expected_ttl_mins,'%2.2f') 'mins [band=' num2str(band_id) ']'];

function d=getFile(d,p )
path = '~/drc/data/'
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


d.rx.names = names( direction==0 );
d.rx.data = data(: , (direction ==0) );
d.rx.names  = d.rx.names (  d.rx.data(end,:) ~=0 );
d.rx.data  = d.rx.data (:,  d.rx.data(end,:) ~=0 )/ p.scale;
d.tx.names = names( direction==1 );
d.tx.data = data(:, (direction ==1) );
d.tx.names  = d.tx.names (  d.tx.data(end,:) ~=0 );
d.tx.data  = d.tx.data (:,  d.tx.data(end,:) ~=0 )/ p.scale;

% remove the burp at the start of our operation:
%start_idx = find ( d.sec  >= end_of_init_time , 1 , 'first');
%idx_keep = [start_idx : size(d.sec,1)];
%tx_data_blurp = d.tx.data(start_idx, :);
%d.tx.data = d.tx.data(idx_keep, :) - repmat ( tx_data_blurp,  size(d.tx.data(idx_keep, :),1 ),1 ) ;
%rx.data_blurp = d.rx.data(start_idx, :);
%d.rx.data = d.rx.data(idx_keep, :) - repmat ( rx.data_blurp,  size(d.rx.data(idx_keep, :),1 ),1 ) ;
%d.sec = d.sec(idx_keep);
d.min_sec = d.sec(1);
d.sec= d.sec - d.min_sec;






function vrc=getVRCScoreFile(vrc,p ,d)
path = '~/drc/data/'
files = dir([path 'vrc-score-*.csv']);
filename = files(end).name

fid=fopen([path filename]);
tline = fgetl(fid);
names = regexp(tline,',','split');
names = names(2:end);

data= dlmread([path filename],',',1,0);
vrc.sec = data(:,1)*1E-6;
vrc.rx.data = data(:,2)/p.scale;
vrc.tx.data = data(:,3)/p.scale;
vrc.sec= vrc.sec - d.min_sec ; % matches the shaper's timestamps
