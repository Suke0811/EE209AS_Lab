classdef MDP    
    %% private constant
    properties (Constant,Access = private) 
        % constant
        
        %randi range
        randiRange = 100;
        %#of attempt
        iteration = 100000;
    end
    
    %% public constant
    properties (Constant) 
        %to specify direction of motion
        FORWARDS = 1;
        BACKWARDS = -1;
        STILL = 0;
        RIGHT = 1;
        LEFT = -1;
        %# of direction (0 to 11)
        dirIdent = 12;
    end
    
    properties
        %2d grid size (length, width)
        L;W;
        %grid matrix
        grid;
        %error probability
        Pe;
        %State space,(Sm = matrix form)
        S;Sm
        %action space
        A;
        %if robot orientation matters on reward
        IsDirectionalRewardOn
    end
    
    properties (Dependent) %getter required
        
    end
    
    methods 
        %% constructor, takes length and width of grid, error probability
        function obj = MDP(L,W,Pe,IsDirectionalRewardOn)
            obj.L = L; obj.W = W; obj.Pe = Pe;
            obj.IsDirectionalRewardOn = IsDirectionalRewardOn;

            obj = obj.generateGridMatrix(); %generate grid matrix
            obj = obj.createStateSpace();   %create state space
            obj = obj.createActionSpace();  %create action space
        end
        
        %% return pre rotation (-1,0,1) 0 means no error rotation
        function errorRotation = getPreRotation(obj)
            randVal = randi(obj.randiRange);
            %evaluate the result
            if(randVal < 100*obj.Pe)
                %0 < x < Pe, -1
                errorRotation = -1;
            elseif(100*obj.Pe <= randVal && randVal < 200*obj.Pe)
                %(Pe < x < 2*Pe), 1
                errorRotation = 1;
            else
                % 1- 2*Pe, 0 (no error)
                errorRotation = 0;
            end    
        end
        
        
        %% move in direction, (In:s, a; Out: s') (s=[x,y,h],a=[dir,turn])
        %if forward(2,3,4 => +x)(8,9,10 =>-x)
        % (11,0,1 => +y)(5,6,7 => -y)
        %turn included
        function sPrime = moveInDirectionAbs(obj,s,a)
            %get orientation of robot
            h = mod(s(3),obj.dirIdent);
            s = reshape(s,1,3);
            if(a(1) == 0) %no motion
                pos_dx = 0; pos_dy = 0;
                a(2) = 0; 
            else
               
                %either forwards or backwards   
                %make sure direction is unit
                a(1) = a(1)/abs(a(1));
            
                switch(h)
                    case {2,3,4} %facing right
                        %check if the robot is at the edge
                        if(s(1) == obj.W-1 && a(1)==1)
                            a(1) = obj.STILL;
                        elseif(s(1) == 0 && a(1)==-1)
                            a(1) = obj.STILL;
                        end
                        
                        pos_dx = a(1); pos_dy = 0;
                        
                    case {8,9,10} %facing left
                        %check if the robot is at the edge
                        if(s(1) == obj.W-1 && a(1)==-1)
                            a(1) = obj.STILL;
                        elseif(s(1) == 0 && a(1)==1)
                            a(1) = obj.STILL;
                        end
                        
                        pos_dx = -a(1); pos_dy = 0;

                    case {11,0,1} %facing up
                        %check if the robot is at the edge
                        if(s(2) == obj.L-1 && a(1)==1)
                            a(1) = obj.STILL;
                        elseif(s(2) == 0 && a(1)==-1)
                            a(1) = obj.STILL;
                        end
                        
                        pos_dx = 0; pos_dy = a(1);

                    case {5,6,7} %facing down
                        %check if the robot is at the edge
                        if(s(2) == obj.L-1 && a(1)==-1)
                            a(1) = obj.STILL;
                        elseif(s(2) == 0 && a(1)==1)
                            a(1) = obj.STILL;
                        end
                        
                        pos_dx = 0; pos_dy = -a(1);

                end
            end
            
            sPrime = [s(1)+pos_dx, s(2)+pos_dy, mod(h+a(2),obj.dirIdent)];
            
        end
        
        %% motion sequence (error rotation, move in direction, turn)
        %In: s, a (current state s=[x,y,h], action a=[dir,turn])
        %Out: s'
        function sPrime = motionSequence(obj,s,a)
           
           %if robot is stand still, no error, otherwise there is
           if(a ~= [obj.STILL, obj.STILL])
           %there is a error in rotation when move
               s(3) = mod(s(3) + obj.getPreRotation(), obj.dirIdent);
           end
           
           %move in direction (a=[direction, turn direction])
           sPrime = obj.moveInDirectionAbs(s,a);

        end
        
        %% Action probability  Psa(s') = f(Pe,s,a,s')
        % In: s,a,s' (s=[x,y,h],a=[forwards,still,backwards],s'=[x,y,h])
        % Out: Psa
        % actually run the motionSequence function to generate Psa
        function Psa = calcActionProbActual(obj,s,a,sPrime)
            % actually attempt motion for iteration times with s,a
            %count if s' occurs
            eventHappened = 0;
            for k1 = 1:obj.iteration
                if(sPrime == obj.motionSequence(s,a))
                    eventHappened = eventHappened + 1;
                end
            end
            Psa = eventHappened / obj.iteration;
        end
        
        %% Action probability  Psa(s') = f(Pe,s,a,s')
        % In: s,a,s' (s=[x,y,h],a=[forwards,still,backwards],s'=[x,y,h])
        % Out: Psa
        function Psa = calcActionProb(obj,s,a,sPrime)
            %there are 3 possible sPrime from the same s and a
            sPrime = reshape(sPrime,1,3);
            if(sPrime == obj.moveInDirectionAbs(s,a))
                %no error pattern
                Psa = 1 - 2*obj.Pe; 
            elseif(sPrime == obj.moveInDirectionAbs([s(1),s(2),s(3)+1],a))
                %orientation shift by +1
                Psa = obj.Pe;
            elseif(sPrime == obj.moveInDirectionAbs([s(1),s(2),s(3)-1],a))
                %orientation shift by -1
                Psa = obj.Pe;
            else
                %otherwise, not possible
                Psa = 0;
            end  
        end 
        
        %% Action probability  Psa(s') = f(Pe,s,a,s')
        % In: s,a (s=[x,y,h],a=[forwards,still,backwards])
        % Out: sPrime=[x y h; x y h; x y h]
        function sPrime = generatePosibleSPrime(obj,s,a)
            %there is 3 possible sPrime from the same s and a
            sPrime(1:3,1) = obj.moveInDirectionAbs(s,a);
            if(obj.Pe ~= 0)
                sPrime(1:3,2) = obj.moveInDirectionAbs([s(1),s(2),s(3)+1],a);
                sPrime(1:3,3) = obj.moveInDirectionAbs([s(1),s(2),s(3)-1],a);
            end
        end
    
        
          %% reward (In: s; Out:reward) (s=[x,y.h])
        function reward = prob2Reward8By8(obj,s)
            if(s(1) == 5 && s(2) == 6)
                %at(5,6) reward 1
                
                if(obj.IsDirectionalRewardOn)
                    %if robot orientation matters
                    if(s(3) == 6) %h = 6(straight down)
                        reward = 1;
                    else
                        reward = 0;
                    end
                else
                    reward = 1;
                end
                
            elseif(s(1) == 3 && (s(2) == 4 || s(2) == 5 || s(2) == 6))
                %(3,4), (3,5), (3,6), reward -10 
                reward = -10;
            elseif(s(1) == 0 || s(2) == 0 || s(1) == 7 || s(2) == 7)
                %if x = 0 or 7, y = 0 or 7, reward -100
                reward = -100;
            else
                %otherwise 0
                reward = 0;
            end
        end
        
    
    end
    

    
    methods (Access = private)
        %% generate Grid matrix, return obj
        function obj = generateGridMatrix(obj)
            obj.grid = zeros(obj.W,obj.L);
        end
        
        %% create state space
        function obj = createStateSpace(obj)
            %size of state space is L*W (1st row: x, 2nd row: y)
            obj.S = zeros(2,obj.L*obj.W*obj.dirIdent);
            obj.Sm = zeros(obj.L,obj.W,obj.dirIdent);
            count=1;
            for k1=1:obj.L
                for k2=1:obj.W
                    for k3=1:obj.dirIdent
                        obj.S(1,count) = k1 - 1;
                        obj.S(2,count) = k2 - 1;
                        obj.S(3,count) = k3 - 1;
                        count = count + 1;
                    end
                end
            end
        end
        
        %% create action space
        function obj = createActionSpace(obj)
            %action space 
            %orientation
            h = [obj.RIGHT, obj.STILL, obj.LEFT];
            %still and all orientations
            A1 = [0;0];
            %forwards and all orientations
            A2 = obj.FORWARDS*ones(1,length(h));
            A2 = [A2;h];
            %backwards and all orientations
            A3 = obj.BACKWARDS*ones(1,length(h));
            A3 = [A3;h];
            
            obj.A = [A1 A2 A3];
        end
    end
end