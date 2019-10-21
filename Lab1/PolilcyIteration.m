classdef PolilcyIteration
    properties
        
    end
    properties %(Access = private)
        %MDP instance
        mdp;
        %MDPUI instance
        ui;
        %init policy
        Pi0;
        %discount
        dis;
        %previous pie
        PiePrev;
        %Value function previous step
        Vpre;
    end
    properties (Constant)
        
    end
    
    methods
        %% constructor
        function obj = PolilcyIteration(L,W,Pe,dis,IsDirectionalRewardOn)
            %Create mdp robot instance
            obj.mdp = MDP(L,W,Pe,IsDirectionalRewardOn);
            obj.ui = MDPUI();
            obj.dis = dis;
            obj.Vpre = zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
            obj.PiePrev = zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
            obj = obj.createInitPolicy();
        end
        
        function obj = createInitPolicy(obj)
            obj.mdp.S;
            obj.mdp.A;
            %populate action that stay current position(a=[0,0])
            obj.Pi0 = obj.mdp.Sm + 1;
            obj.PiePrev = obj.Pi0;
        end
        
        %% calculate value function at a state
        %In:s=[x,y,h], Vpre=matrix of W*L*H (value function matrix from previous step)
        %    Pie (policy ti evaluate)
        %Out:v (max value possible to gain at the state)
        function v = valueFunctionAtState(obj,s,Vpre,Pie)
            %evaluate current reward, and sum past value with discount
            %calculate valufunction for each action posible
            vTemp = zeros(1,size(obj.mdp.A,2));
            
            %extract current action
            aIndex = Pie(s(1)+1,s(2)+1,s(3)+1);
            aCurrent = obj.mdp.A(:,aIndex);
            
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
                vTemp = vTemp + probTemp*(rTemp + obj.dis*Vsprime);
            end
            
            %find max value and corresponding policy
            v = max(vTemp);
        end
        
        %% refine policy, 
        %In: s=[x,y,h], Value matrix; Out: v,a(value and better action)
        function [v,a] = refinePolicy(obj,s,Vpre)
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
        
        %% calculate one step of value function 
        %In: Vpre=matrix of W*L*H (value function matrix from previous step)
        %Out: V, (the same size as Vpre, value matrix)
        function Vpie = calcValue(obj,Vpre,Pie)
            count = 1;
            %value matrix
            Vpie=zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
            
            for k1=1:obj.mdp.W
                for k2=1:obj.mdp.L
                    for k3=1:obj.mdp.dirIdent
                        %pick one state to evaluate value function
                        %S contains all posible state vectors S=s0,s2,s3...
                        sCurrent = obj.mdp.S(:,count);
                        %get value function at that point
                        [Vpie(k1,k2,k3)] = obj.valueFunctionAtState(sCurrent,Vpre,Pie);
                        count = count + 1;
                    end
                end
            end
        end
            
        %% calculate one step of value function and get best policy at that step
        %In: Vpre=matrix of W*L*H (value function matrix from previous step)
        %Out: V,Pie (the same size as Vpre, value and policy matrix)
        function [Vpie,Pie] = calcValueAndPolicy(obj,Vpre,Pie)
            count = 1;
            %value matrix
            Vpie=zeros(obj.mdp.L,obj.mdp.W,obj.mdp.dirIdent);
            
            for k1=1:obj.mdp.W
                for k2=1:obj.mdp.L
                    for k3=1:obj.mdp.dirIdent
                        %pick one state to evaluate value function
                        %S contains all posible state vectors S=s0,s2,s3...
                        sCurrent = obj.mdp.S(:,count);
                        %get value function at that point
                        [Vpie(k1,k2,k3)] = obj.valueFunctionAtState(sCurrent,Vpre,Pie);
                        count = count + 1;
                    end
                end
            end
            
            count = 1;
            for k1=1:obj.mdp.W
                for k2=1:obj.mdp.L
                    for k3=1:obj.mdp.dirIdent
                        %pick one state to evaluate value function
                        %S contains all posible state vectors S=s0,s2,s3...
                        sCurrent = obj.mdp.S(:,count);
                        %get best value function and corresponding Action
                        [Vpie(k1,k2,k3),Pie(k1,k2,k3)] = obj.refinePolicy(sCurrent,Vpie);
                        count = count + 1;
                    end
                end
            end
        end
        
        %% calculate optimal value and policy
        function [V,Pie] = calcOptimalValueAndPolicy(obj)
            %do_while loop
            while(true)
                %cal current value and policy
                [Vcur,Pie] = obj.calcValueAndPolicy(obj.Vpre,obj.PiePrev);
                
                %loop exit condition
                if(Pie == obj.PiePrev)
                    break;
                end
                
                obj.Vpre = Vcur;
                obj.PiePrev = Pie;
            end
            %out put
            V = Vcur;
            
        end
        
         %% show init policy trajectory with init state, s0
        function showPolicyTrajectory(obj,s0,Pie)
            %clear map
            obj.ui.clearTrajectory();
            %draw init pos
            obj.ui.drawOnlineTrajectory(s0);
            s = s0;
            %generate trajectory
            for k = 1:100
                %get corresponding action at the cell
                aIndex = Pie(s(1)+1,s(2)+1,s(3)+1);
                %perform action
                s = obj.mdp.motionSequence(s,obj.mdp.A(:,aIndex));
                %draw new position
                obj.ui.drawOnlineTrajectory(s);
            end
        end
    end
    
    methods (Access = private)
        
    end
    
end