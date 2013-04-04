function main()
disp('dfdf')
for i=1:100
  i
system = mod(i,5);
importance = mod(i,3);
frequency  = 0;
value = ['Message here ' num2str(i) ' ' num2str(mod(i,5)) ...
  ' ' num2str(mod(i,3))];
send_status(system, importance,frequency, value);
pause(0.25)
end
