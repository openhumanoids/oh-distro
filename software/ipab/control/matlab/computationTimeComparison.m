function computationTimeComparison(statVars1, StatVars2)
  close all
  load(statVars1);
  IKtimes = {statVars.IKTimes'};
  CollisionTimesOM = statVars.collisionTimes;
  EEcollisions = {CollisionTimesOM(1,CollisionTimesOM(2,:) <= 2)'};
  WBcollisions = {CollisionTimesOM(1,CollisionTimesOM(2,:) > 2)'};
  load(StatVars2);
  IKtimes{2} = statVars.IKTimes';
  CollisionTimesNM = statVars.collisionTimes;
  EEcollisions{2} = CollisionTimesNM(1,CollisionTimesNM(2,:) <= 2)';
  WBcollisions{2}= CollisionTimesNM(1,CollisionTimesNM(2,:) > 2)';
  
  nIKOM = size(IKtimes{1}, 1);
  nEEOM = size(EEcollisions{1}, 1);
  nWBOM = size(WBcollisions{1}, 1);
  nIKNM = size(IKtimes{2}, 1);
  nEENM = size(EEcollisions{2}, 1);
  nWBNM = size(WBcollisions{2}, 1);
  
  [~, idx] = min([nIKOM nIKNM]);
  IKtimes{idx} = [IKtimes{idx}; NaN(abs(nIKOM-nIKNM), 1)];
  IKtimes = cell2mat(IKtimes);
  [~, idx] = min([nEEOM nEENM]);
  EEcollisions{idx} = [EEcollisions{idx}; NaN(abs(nEEOM-nEENM), 1)];
  EEcollisions = cell2mat(EEcollisions);
  [~, idx] = min([nWBOM nWBNM]);
  WBcollisions{idx} = [WBcollisions{idx}; NaN(abs(nWBOM-nWBNM), 1)];
  WBcollisions = cell2mat(WBcollisions);
  
  labels = {'1', '2'};
  aboxplot(IKtimes, 'OutlierMarker', '+', 'colorrev',true, 'labels', labels);
  title('IK solution times')
  annotation('textbox', [.34,0.78,0.4,0.1], 'FitBoxToText', 'On',...
    'String', sprintf('n IK runs (old model): %d\nn IK runs (new model): %d', nIKOM, nIKNM))
  pause(1)
  % print('-depsc', 'IKtimes.eps')
  
  figure;
  aboxplot(EEcollisions, 'OutlierMarker', '+', 'colorrev',true, 'labels', labels);
  title('EE collision check times')
  annotation('textbox', [.34,0.78,0.4,0.1], 'FitBoxToText', 'On',...
    'String', sprintf('n checks 1: %d\nn checks 2: %d', nEEOM, nEENM))
  pause(1)
  % print('-depsc', 'EEcoliisions.eps')
  
  figure;
  aboxplot(WBcollisions, 'OutlierMarker', '+', 'colorrev',true, 'labels', labels);
  title('Whole-body collision check times')
  annotation('textbox', [.34,0.78,0.4,0.1], 'FitBoxToText', 'On',...
    'String', sprintf('n checks 1: %d\nn checks 2: %d', nWBOM, nWBNM))
  pause(1)
  % print('-depsc', 'WBcollisions.eps')
  
  fprintf(['DATA 1:\n'...
    'n IK runs: %d\n'...
    'n EE collision runs: %d\n'...
    'n WB collision runs: %d\n'...
    'IK toal time: %.2f\n'...
    'EE collision total time: %.2f\n'...
    'WB collision total time: %.2f\n'],...
    nIKOM, nEEOM, nWBOM, sum(IKtimes(1:nIKOM, 1)), sum(EEcollisions(1:nEEOM, 1)), sum(WBcollisions(1:nWBOM, 1)))
  fprintf(['DATA 2:\n'...
    'n IK runs: %d\n'...
    'n EE collision runs: %d\n'...
    'n WB collision runs: %d\n'...
    'IK toal time: %.2f\n'...
    'EE collision total time: %.2f\n'...
    'WB collision total time: %.2f\n'],...
    nIKNM, nEENM, nWBNM, sum(IKtimes(1:nIKNM,2)), sum(EEcollisions(1:nEENM,2)), sum(WBcollisions(1:nWBNM,2)))
end