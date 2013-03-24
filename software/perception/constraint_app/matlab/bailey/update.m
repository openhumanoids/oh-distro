function update(z,R,idf)
global XX PX

for i=1:length(idf)
    [XX,PX] = unscented_update(@observe_model, @observediff, XX,PX, z(:,i),R, idf(i));
end

%

function dz = observediff(z1, z2)
dz = z1-z2;
dz(2,:) = pi_to_pi(dz(2,:));
