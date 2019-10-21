%error probability
Pe = 0.0;
%discount factor
dis = 0.9;
%tolerance
tol = 0.01;
%init state
s0 = [1 6 6];

%number of run 
numTest = 1;

%create instances
val = ValueIteration(8,8,Pe,dis,false);
pol = PolilcyIteration(8,8,Pe,dis,false);

time = 0;
for k=1:numTest
    tic
    %get optimal value and policy
    [V1,Pie1] = val.calcOptimalValueAndPolicy(tol);
    time = time + toc;
end
%average run time
display(time/numTest);
figure(1);
%show robot trajectory based on the policy
val.showPolicyTrajectory(s0,Pie1);


time = 0;
for k=1:numTest
    tic
    %get optimal value and policy
    [V2,Pie2] = pol.calcOptimalValueAndPolicy();
    time = time + toc;
end
%average run time
display(time/numTest);

figure(2);
%show robot trajectory based on the policy
pol.showPolicyTrajectory(s0,Pie2);


% Pe = 0.25;
% 
% 
% val = ValueIteration(8,8,Pe,dis,true);
% pol = PolilcyIteration(8,8,Pe,dis,true);
% 
% time = 0;
% for k=1:numTest
%     tic
%     %get optimal value and policy
%     [V1,Pie1] = val.calcOptimalValueAndPolicy(tol);
%     time = time + toc;
% end
% %average run time
% display(time/numTest);
% 
% figure(3);
% %show robot trajectory based on the policy
% val.showPolicyTrajectory(s0,Pie1);
% 
% time = 0;
% for k=1:numTest
%     tic
%     %get optimal value and policy
%     [V2,Pie2] = pol.calcOptimalValueAndPolicy();
%     time = time + toc;
% end
% %average run time
% display(time/numTest);
% 
% figure(4);
% %show robot trajectory based on the policy
% pol.showPolicyTrajectory(s0,Pie2);