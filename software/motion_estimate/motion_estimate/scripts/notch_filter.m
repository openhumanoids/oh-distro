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

if (1==0)
x=0.7*sin(2*pi*84*t)+sin(2*pi*120*t )  ;% +0.1*randn(size(t) );
else
  x= 0*ones(1000,1);
  x_read =load('accel3.txt')
  x(1:size(x_read,1)) = x_read;
end
x_original  = x;

figure
plot(x)

figure
[S,F,T,P] = spectrogram(x , 256,255,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds)'); ylabel('Hz');

figure
subplot(2,1,1);
plot(Fs*t(1:50),x(1:50));

pump_hz = 85;
if (1==1)
  wo = pump_hz/(1000/2); bw = wo;%wo/60;
  [b,a] = iirnotch(wo,bw);
  x=filter(b,a,x);
end

if (1==1)
  wo = 2*85/(1000/2); bw = wo;
  [b,a] = iirnotch(wo,bw);
  x=filter(b,a,x);  
end
if (1==1)
  wo = 4*85/(1000/2); bw = wo;
  [b,a] = iirnotch(wo,bw);
  x=filter(b,a,x);  
end


if (1==0)
  x_dft_original = fft(x_original);
  x_dft = fft(x);
  freq = 0:(Fs/length(x)):500;
  subplot(211)
  plot(freq,abs(x_dft_original(1:501)).^2)
  subplot(212)
  plot(freq,abs(x_dft(1:501)).^2);
end

%Wo = 50/(1000/2);  BW = Wo/60;
%[b,a] = iirnotch(Wo,BW);  
fvtool(b,a);

% Wo = 450/(1000/2);  BW = Wo/35;
%[b,a] = iirnotch(Wo,BW);  
%fvtool(b,a);





figure 
plot(x)


figure
[S,F,T,P] = spectrogram(x , 256,255,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds)'); ylabel('Hz');
