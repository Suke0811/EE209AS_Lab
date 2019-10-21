classdef MDPUI
   properties
       %mesh grid
       X,Y;
       %color map
       C;mymap;
   end
   
   methods
       function obj = MDPUI()
           obj = obj.create8By8Field();
           obj.plotField();
       end
       
       function obj = create8By8Field(obj)
          [obj.X,obj.Y] = meshgrid(0:8);
          obj.C = [1 1 1 1 1 1 1 1 5;
                   1 0 0 0 0 0 0 1 5;
                   1 0 0 0 0 0 0 1 5;
                   1 0 0 0 0 0 0 1 5;
                   1 0 0 4 0 0 0 1 5;
                   1 0 0 4 0 0 0 1 5;
                   1 0 0 4 0 2 0 1 5;
                   1 1 1 1 1 1 1 1 5;
                   5 5 5 5 5 5 5 5 5];
               obj.mymap = [1 1 1;1 0 0; 0 1 0; 0 0 1; 1 1 0; 0 0 0];
       end
       
       function plotField(obj)
           pcolor(obj.X,obj.Y,obj.C);
           colormap(obj.mymap);
           axis square;
       end
       
       %% draw robot position (In: s =[x,y,h])
       function drawRobot(obj,s)
           %shift x,y position such that the robot comes at the center of
           %the cell
           s(1) = s(1) + 0.5; s(2) = s(2) + 0.5;
           %convert h into degree
           th = 30 * s(3);
           %calculate vectors for arrow
           x = [s(1)-0.25*sind(th) s(1)+0.4*sind(th)];
           y = [s(2)-0.25*cosd(th) s(2)+0.4*cosd(th)];
           %draw arrow with davinci
           davinci( 'arrow', 'X', x, 'Y', y,...
               'Shaft.Type', 'rectangle', 'Head.Length', 0.4);   
       end
       
       function drawRobotCurrentState(obj,s)
          %renew field
          hold off; 
          obj.plotField();
          hold on;
          obj.drawRobot(s); 
       end
       
       function drawOnlineTrajectory(obj,s)
           hold on;
           obj.drawRobot(s);
       end
       
       function clearTrajectory(obj)
           hold off;
          obj.plotField(); 
       end
   end
   
   methods (Access = private)
    
       
   end
end