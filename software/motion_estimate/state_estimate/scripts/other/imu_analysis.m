%86 peak 172, 262 344 426

close all

d =load ('imu_data');
% utime drot[3] linecc[3]

t0 = d(1,1)
figure
plot(  (d(:,1)-t0)*1E6, d(:,5),'.-')

sig1=d(:,6)

figure
[S,F,T,P] = spectrogram(sig1 , 1024,896,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds)'); ylabel('Hz');

%86Hz?

% 2. notch filtering
x = sig1(3001:4000)

figure
subplot(2,1,1);
plot(Fs*t(1:50),x(1:50));

if (1==0)
  wo = 86/(1000/2); bw = wo/5;
  [b,a] = iirnotch(wo,bw);
else
  [b,a] = butter(4, 0.01)
end  
  
y=filter(b,a,x);



if (1==1)
  wo = 44/(1000/2); bw = wo/5;
  [b,a] = iirnotch(wo,bw);
else
  [b,a] = butter(4, 0.1)
end    
y=filter(b,a,y);

if (1==1)
  wo = 49/(1000/2); bw = wo/5;
  [b,a] = iirnotch(wo,bw);
else
  [b,a] = butter(4, 0.1)
end    
y=filter(b,a,y);

ydft = fft(y);
xdft = fft(x);
freq = 0:(Fs/length(x)):500;
subplot(211)
plot(freq,abs(xdft(1:501)).^2)
subplot(212)
plot(freq,abs(ydft(1:501)).^2);

figure
subplot(2,1,1)
plot(x)
hold on
plot(y,'r')
subplot(2,1,2)
plot(y)

keyboard




wo = 86/(1000/2); bw = wo/35;
[b,a] = iirnotch(wo,bw);

x= x(1:1000)

y=filter(b,a,x);
ydft = fft(y);
xdft = fft(x);
freq = 0:(Fs/length(x)):500;
figure
subplot(211)
plot(freq,abs(xdft(1:501)).^2)
subplot(212)
plot(freq,abs(ydft(1:501)).^2);



 Wo = 450/(1000/2);  BW = Wo/35;
[b,a] = iirnotch(Wo,BW);  
fvtool(b,a);



%S = spectrogram(x,window,noverlap,nfft,fs) uses fs sampling
%frequency in Hz. If fs is specified as empty [],
%it defaults to 1 Hz.