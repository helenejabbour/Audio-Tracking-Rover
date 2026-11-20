function R = rover_reference_design(runFolder)
% Independent base-MATLAB reference; compare against a saved Python validation.
% Run from repo root: addpath('Matlab'); R=rover_reference_design('Runs/...');
% No hardware I/O. Does not claim the legacy Simulink model is validated.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin==0
    cfg=jsondecode(fileread(fullfile(root,'Config','simulation.json')));
else
    cfg=jsondecode(fileread(fullfile(runFolder,'config.json')));
end
a=cfg.nominal_speed/cfg.desired_range;
A=[0 0 0;0 0 a;0 0 0];B=[-1 0;0 -1;0 -1];C=[1 0 0;0 1 0];
Aa=[A zeros(3,1);1 0 0 0];Ba=[B;0 0];
Z=expm([Aa Ba;zeros(2,6)]*cfg.dt);Ad=Z(1:4,1:4);Bd=Z(1:4,5:6);
Q=diag(cfg.lqr_q);Ru=diag(cfg.lqr_r);P=Q;
for k=1:20000
    Pnext=Ad'*P*Ad-Ad'*P*Bd*((Ru+Bd'*P*Bd)\(Bd'*P*Ad))+Q;
    if norm(Pnext-P,'fro')<1e-12*max(1,norm(P,'fro')),P=Pnext;break;end
    P=Pnext;
end
assert(k<20000,'DARE iteration did not converge');
K=(Ru+Bd'*P*Bd)\(Bd'*P*Ad);
Rc=[B A*B A^2*B];Ro=[C;C*A;C*A^2];
assert(rank(Rc)==3 && rank(Ro)==3);
F=Ad-Bd*K;assert(max(abs(eig(F)))<1);
% vec(F' V F - V)=-vec(I), using base MATLAB.
V=reshape((eye(16)-kron(F',F'))\reshape(eye(4),16,1),4,4);
assert(min(eig((V+V')/2))>0);
R=struct('A',A,'B',B,'C',C,'Ad',Ad,'Bd',Bd,'K',K,'poles',eig(F),'V',V);
disp('LQI K (columns range, bearing, relative heading, range integral):');disp(K);
disp('Discrete closed-loop poles:');disp(eig(F));
fprintf('Open-loop A has a nontrivial zero Jordan block: Lyapunov unstable, not BIBO stable.\n');
if nargin>0
    ref=jsondecode(fileread(fullfile(runFolder,'analysis.json')));
    err=max(abs(K-ref.K),[],'all');fprintf('Max difference vs Python K: %.3g\n',err);
    assert(err<1e-7,'Python/MATLAB gain mismatch');
end
end
