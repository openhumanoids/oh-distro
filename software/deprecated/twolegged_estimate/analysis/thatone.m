

data = dlmread('2err.txt',',',0,1);

data = data(1:(end-2),1:(end-1));

plot(data)