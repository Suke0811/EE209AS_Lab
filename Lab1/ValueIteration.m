classdef ValueIteration
    properties
        
    end
    properties %(Access = private)
        %MDP instance
        mdp;
        %MDPUI instance
        ui;
        %discount factor
        dis;
        %Value function previous step
        Vpre;
        %current Value function
        %  Vcur;
    end
    properties (Constant)
        
    end
    
    methods
        %% constructor
        function obj = ValueIteration(L,W,Pe,dis,IsDirectionalRewardOn)
            %Create mdp robot instance
            obj.mdp = MDP(L,W,Pe,IsDirectionalRewardOn);
            obj.ui = MDPUI();
            obj.dis = dis;
            obj.Vpre = zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
        end
        
        %% calculate value function at a state
        %In:s=[x,y,h], Vpre=matrix of W*L*H (value function matrix from previous step)
        %Out:v (max value possible to gain at the state)
        %   :a (action that gains max value)
        function [v,a] = valueFunctionAtState(obj,s,Vpre)
            %evaluate current reward, and sum past value with discount
            %calculate valufunction for each action posible
            vTemp = zeros(1,size(obj.mdp.A,2));
            
            for ka = 1:size(obj.mdp.A,2)
                %extract one action
                aCurrent = obj.mdp.A(:,ka);
                %generate possible action
                sPrime = obj.mdp.generatePosibleSPrime(s,aCurrent);
                
                %calculate partial value function for each posible s'
                %Note: impossible s' has zero probability => zero value
                for ksp = 1:size(sPrime,2)
                    %get probability to s' with aCurrent(action)
                    probTemp = obj.mdp.calcActionProb(s,aCurrent,sPrime(:,ksp));
                    %get reward at s'
                    rTemp = obj.mdp.prob2Reward8By8(sPrime(:,ksp));
                    %get value function from previous step at s'
                    %h are from 0, so index is +1
                    Vsprime = Vpre(sPrime(1,ksp)+1,sPrime(2,ksp)+1,sPrime(3,ksp)+1);
                    %sum all value with the same action
                    %Psa*(r(s')+dis*V(s'))
                    vTemp(ka) = vTemp(ka) + probTemp*(rTemp + obj.dis*Vsprime);
                end
            end
            
            %find max value and corresponding policy
            [v,maxIndex] = max(vTemp);
            a = maxIndex;
        end
        
        
        %% calculate one step of value function and get best policy at that step
        %In: Vpre=matrix of W*L*H (value function matrix from previous step)
        %Out: V,Pie (the same size as Vpre, value and policy matrix)
        function [V,Pie] = calcValueAndPolicy(obj,Vpre)
            count = 1;
            %value matrix
            V=zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
            %policy
            Pie=zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
            
            for k1=1:obj.mdp.W
                for k2=1:obj.mdp.L
                    for k3=1:obj.mdp.dirIdent
                        %pick one state to evaluate value function
                        %S contains all posible state vectors S=s0,s2,s3...
                        sCurrent = obj.mdp.S(:,count);
                        %get value function and corresponding Action
                        [V(k1,k2,k3),Pie(k1,k2,k3)] = obj.valueFunctionAtState(sCurrent,Vpre);
                        count = count + 1;
                    end
                end
            end
        end
        
        %% calculate optimal value and policy
        function [V,Pie] = calcOptimalValueAndPolicy(obj,tolerance)
            %do_while loop
            while(true)
                %cal current value and policy
                tic
                [Vcur,Pie] = obj.calcValueAndPolicy(obj.Vpre);
                tac
                %loop exit condition
                if(abs(Vcur - obj.Vpre) < tolerance)
                    break;
                end
                
                obj.Vpre = Vcur;
            end
            %out put
            V = Vcur;
            
        end
        
        %% show policy trajectory with init state, s0
        function showPolicyTrajectory(obj,s0,Pie)
            %clear map
            obj.ui.clearTrajectory();
            %draw init pos
            obj.ui.drawOnlineTrajectory(s0);
            s = s0;
            %generate trajectory
            for k = 1:100
                %get corresponding action
                aIndex = Pie(s(1)+1,s(2)+1,s(3)+1);
                s = obj.mdp.motionSequence(s,obj.mdp.A(:,aIndex));
                obj.ui.drawOnlineTrajectory(s);
            end
        end
        
        
    end
    
    methods (Access = private)
        
    end
    
end