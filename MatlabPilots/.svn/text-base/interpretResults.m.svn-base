function [numErrors] = interpretResults(array_label, realPredVector) 
% This function produces error numbers from the result

indShotChange = find(array_label==1);
indShotPred = find(realPredVector);

numErrors = 0;
for n=1:length(indShotChange)
  [tmp, ind] = min( abs( indShotChange(n) - indShotPred));
  if indShotPred(ind) == indShotChange(n)
  else
    numErrors = numErrors + 1;
    indShotChange(n)
  end
end

fprintf('Number of shot changes not found: %d', numErrors);

