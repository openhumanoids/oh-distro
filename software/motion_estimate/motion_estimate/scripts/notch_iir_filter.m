function notch_iir_filter()
close all
% FIR filters generally require a much higher order than IIR.
% ~84-85Hz signal

% 0. Filter design
%wo = 84/(1000/2);  bw = wo;%wo/1;
%[b,a] = iirnotch(wo,bw);
%fvtool(b,a);

% z-vector accel:
%x =load('/home/mfallon/data/atlas/accel_signals/accel3.txt');
x =load('/home/mfallon/data/atlas/accel_signals/accel_z.txt');





% comparison to pure tone signal
%figure
%plot(x(7:end))
%hold on
%Fs=1000;
%T=1/Fs;
%t=(0:Fs-1)*T;
%x2=0.2+ 0.07*sin(2*pi*84*(t+10) ) ;
%plot(x2,'r')

% was : 85
main_freq = 87


% a single notch filter
harmonic_freq = main_freq *4;
wo = harmonic_freq/(1000/2); bw = wo;%wo/60;
[b,a] = iirnotch(wo,bw);
y_single=filter(b,a,x);


% matlab and diy calculation:
y = notch_cascade_matlab(x,main_freq ,3);
y_mf = notch_cascade_diy(x,main_freq ,3);


% comparison plot:
figure
plot(x,'.')
hold on
plot(y,'r.')
plot(y_mf,'gx')
plot(y_single,'kx')

% comparison spectrogram
figure
[S,F,T,P] = spectrogram(x , 256,255,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds) original'); ylabel('Hz');

figure
[S,F,T,P] = spectrogram(y , 256,255,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds) - cascade'); ylabel('Hz');

figure
[S,F,T,P] = spectrogram(y_mf , 256,255,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds) - mfallon'); ylabel('Hz');

figure
[S,F,T,P] = spectrogram(y_single , 256,255,256,1E3);
surf(T,F,10*log10(P),'edgecolor','none'); axis tight; 
view(0,90);
xlabel('Time (Seconds) - single 4*pump'); ylabel('Hz');

function y = notch_cascade_matlab(x,pump_freq,harmonic)
y = x;
for i=1:harmonic
  harmonic_freq = pump_freq*2^(i-1);
  wo = harmonic_freq/(1000/2); bw = wo;%wo/60;
  [b,a] = iirnotch(wo,bw);
  y=filter(b,a,y);  
end

function y = notch_cascade_diy(x,pump_freq,harmonic)
y =zeros(size(x))

% need to maintain a stuct for each iteration: prefer to have a class
for i=1:harmonic
  coeffs(i).xx=[0,0];
  coeffs(i).yy=[0,0];
end

for j=1:size(x,1)
  input = x(j);
  for i=1:harmonic
    harmonic_freq = pump_freq*2^(i-1);
    wo = harmonic_freq/(1000/2); bw = wo;%wo/60;
    [b,a] = secondorderNotch_mfallon(wo,bw);
    [input, coeffs(i).xx, coeffs(i).yy]=filter_diy(b,a,input, coeffs(i).xx, coeffs(i).yy);  
  end
  
  y(j) = input;
end

function [y, xx,yy] = filter_diy(b,a,x, xx, yy)
xx_temp = [x, xx(1:2)];
yy_temp = [0, yy(1:2)];
y =sum( xx_temp.*b  - yy_temp.*a);
yy= [y , yy(1)];
xx= [x , xx(1)] ;
