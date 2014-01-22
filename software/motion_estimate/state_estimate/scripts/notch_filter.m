% x - 1000 samples
% 
% 120Hz signal
% 50Hz signal
% 
% notch is applied at 0.1 on a normalized scale 
% on the range [0,500Hz]
% ... removes 50Hz signal


close all

Fs=1000;
T=1/Fs;
t=(0:Fs-1)*T;



x=0.7*sin(2*pi*50*t)+sin(2*pi*120*t)  ;% +0.1*randn(size(t) );
figure
plot(x)

figure
subplot(2,1,1);
plot(Fs*t(1:50),x(1:50));
wo = 50/(1000/2); bw = wo/60;
[b,a] = iirnotch(wo,bw);
y=filter(b,a,x);
ydft = fft(y);
xdft = fft(x);
freq = 0:(Fs/length(x)):500;
subplot(211)
plot(freq,abs(xdft(1:501)).^2)
subplot(212)
plot(freq,abs(ydft(1:501)).^2);


%Wo = 50/(1000/2);  BW = Wo/60;
%[b,a] = iirnotch(Wo,BW);  
fvtool(b,a);

% Wo = 450/(1000/2);  BW = Wo/35;
%[b,a] = iirnotch(Wo,BW);  
%fvtool(b,a);


figure
[S,F,T,P] = spectrogram(x , 256,255,256,1E4);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds)'); ylabel('Hz');