log = load('sandia_current_log')

plot( log(:,[2:4]) )

win = 100
for i=win+1:size(log,2)
  keyboard
  log(i-win:i,2)
  
end