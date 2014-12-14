%% adjust all plots

%plot at so many sigma
sigma_plot = 3;

figs = findobj('Type','figure');

% figure(1)
t= 1:5000;

    [x,y] = ginput();
    
    start = find(t>x(1),1); %start position in x
    stop = length(t)-find(fliplr(t)<x(2),1); % stop position in x
    spa=4;
    spb=1;

for k=1:length(figs)
    figure(figs(k));

    for n=1:spa
        subplot(spa,spb,n);
        
        v=axis;

        y=get(findobj(get(gca,'Children'),'type','line'),'ydata');
        Ystd = [];
        for n_lin = 1:length(y)
             Ystd = [Ystd std(y{n_lin}(1,start:stop))];

        end
        v(3) = -sigma_plot*max(Ystd);
        v(4) = sigma_plot*max(Ystd);

        for n_lin = 1:length(y)
            if (min(y{n_lin}(1,start:stop)) < v(3))
                v(3) = min(y{n_lin}(1,start:stop))*1.1;
            end
            if (max(y{n_lin}(1,start:stop)) > v(4))
                v(4) = max(y{n_lin}(1,start:stop))*1.1;
            end
        end

        axis([x' v(3:4)]);

    end
end
