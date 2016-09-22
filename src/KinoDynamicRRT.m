%
% THIS IS AN IMPLEMENTATION OF KINODYNAMIC RAPIDLY EXPLORING RANDOM TREES (RRT)
% PROPOSED BY STEVEN M LAVALLE IN 1998
%
% COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE
%
% AUTHOR : LAKSHMAN KUMAR
% AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
% EMAIL : LKUMAR93@UMD.EDU
% LINKEDIN : WWW.LINKEDIN.COM/IN/LAKSHMANKUMAR1993
%
% THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THE MIT LICENSE
% THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF
% THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
% 
% BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
% BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
% CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
% CONDITIONS.
%

function KinoDynamicRRT
 
    clc;
    
    figure;
    
    pause(0);

    ShowMap;

    RobotLocationXInit = 250;
    RobotLocationYInit = 250;
    RobotOrientationInit = 0;

    RobotLocationXGoal = 50;
    RobotLocationYGoal = 50;
    RobotOrientationGoal = 90;

    MaxNumberOfIterations = 1000;
    MaxXYThreshold = 5;
    MaxOrientationThreshold = 15;
    MaxStepSize = 300;

    MinimumX = 0; MaximumX = 300;
    MinimumY = 0; MaximumY = 300;

    MinimumOrientation = 0 ; MaximumOrientation = 360;

    Velocity = 1;
    SteeringAngleMax = 45;
    
    
    NumberOfTrials = 20;
    SimulationTime = 15;
    SamplingInterval = 0.1;
    CarLength = 20;  
    GoalBiasRegionStepSize = 50;
    
    Alternator = 0;
    
    title('Pick the Starting Position')
    
    while true;
        
        
     [RobotLocationXInit,RobotLocationYInit] =ginput(1);
     RobotOrientationInit = round((MaximumOrientation-MinimumOrientation)*rand + MinimumOrientation );
     
       if ( IsThereAnObstacle(RobotLocationXInit,RobotLocationYInit) == 0 && CheckCollisionDetectionCorners(RobotLocationXInit,RobotLocationYInit,RobotOrientationInit) == 0 && CheckCollisionDetection(RobotLocationXInit,RobotLocationYInit,RobotOrientationInit) == 0 )
          break ;
       else
           title({'The selected starting position is not a valid configuration.' ; ' Select the starting position again'})           
       end 
       
    end
    
    
    RobotLocationX = RobotLocationXInit;
    RobotLocationY = RobotLocationYInit;
    RobotOrientation = RobotOrientationInit;

    H1 = DrawRobot(RobotLocationX,RobotLocationY,RobotOrientation,'cyan');
    
    pause(0);
  
    hold on;
    
    title('Pick the Goal Position')
    
     while true;
        
        
     [RobotLocationXGoal,RobotLocationYGoal] =ginput(1);
     RobotOrientationGoal = round((MaximumOrientation-MinimumOrientation)*rand + MinimumOrientation );
     
       if ( IsThereAnObstacle(RobotLocationXGoal,RobotLocationYGoal) == 0 && CheckCollisionDetectionCorners(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal) == 0 && CheckCollisionDetection(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal) == 0 )
          break ;
       else
           title({'The selected goal position is not a valid configuration';' Select the goal position again'});           
       end 
       
     end
    
    title('KinoDynamic RRT')    
    
    
    H2 = DrawRobot(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal,'magenta');
    
    pause(0);
    
    legend([H1 H2],{'Starting Position','Goal Position'},'Location','eastoutside');
    
    tree.vertex(1).x = RobotLocationXInit;
    tree.vertex(1).y = RobotLocationYInit;
    tree.vertex(1).o = RobotOrientationInit;

    tree.vertex(1).PreviousX = RobotLocationXInit;
    tree.vertex(1).PreviousY = RobotLocationYInit;
    tree.vertex(1).PreviousOrientation = RobotOrientationInit;
    tree.vertex(1).Distance=0;
    tree.vertex(1).ind = 1; tree.vertex(1).indPrev = 0;

    tic;


    for iter = 2:MaxNumberOfIterations
        
        
        str = sprintf('Number Of Iterations = %d ',iter);
        TimeElapsed =  sprintf('Time Elapsed = %f seconds ',toc);
        
        title({'KinoDynamic RRT';str;TimeElapsed});


            while true    
                while true    

                Alternator = Alternator + 1;

                if mod(Alternator,7) == 0
                                RX = (RobotLocationXGoal - (GoalBiasRegionStepSize/2));
                                RandX = (GoalBiasRegionStepSize)*rand;

                                if RX < MinimumX
                                    RX = MinimumX ;
                                else
                                    if RX > (MaximumX - GoalBiasRegionStepSize)
                                        RX = MaximumX - GoalBiasRegionStepSize;                                       
                                    end
                                end


                                RandomVariableX = round( RandX + RX );

                                if RandomVariableX < MinimumX
                                    RandomVariableX = MinimumX;
                                else
                                    if RandomVariableX > MaximumX
                                        RandomVariableX = MaximumX;
                                    end
                                end

                                RY = (RobotLocationYGoal - (GoalBiasRegionStepSize/2));
                                RandY = (GoalBiasRegionStepSize)*rand;

                                if RY < MinimumY
                                    RY = MinimumY ;
                                else
                                    if RY > (MaximumY - GoalBiasRegionStepSize)
                                        RY = MaximumY - GoalBiasRegionStepSize;
                                    end
                                end

                                RandomVariableY = round( RandY + RY );

                                if RandomVariableY < MinimumY
                                    RandomVariableY = MinimumY;
                                else
                                    if RandomVariableY > MaximumY
                                        RandomVariableY = MaximumY;
                                    end
                                end

                                RO = (RobotOrientationGoal - (GoalBiasRegionStepSize/2));
                                RandO = (GoalBiasRegionStepSize)*rand;

                                if RO < MinimumOrientation
                                    RO = MinimumOrientation ;
                                else
                                    if RO > (MaximumOrientation - GoalBiasRegionStepSize)
                                        RO = MaximumOrientation - GoalBiasRegionStepSize;
                                    end
                                end

                                RandomVariableO = round( RandO + RO );

                                if RandomVariableO < MinimumOrientation
                                    RandomVariableO = MinimumOrientation;
                                else
                                    if RandomVariableO > MaximumOrientation
                                        RandomVariableO = MaximumOrientation;
                                    end
                                end


                else

                    if mod(Alternator,4) == 0
                          RandomVariableX = RobotLocationXGoal ;
                          RandomVariableY = RobotLocationYGoal ;
                          RandomVariableO = RobotOrientationGoal;
                    else
                        RandomVariableX = round(MaxStepSize*rand); 
                        RandomVariableY = round(MaxStepSize*rand); 
                        RandomVariableO = round((MaximumOrientation-MinimumOrientation)*rand + MinimumOrientation );
                    end
                end
                
                if ( IsThereAnObstacle(RandomVariableX,RandomVariableY) == 0 && CheckCollisionDetectionCorners(RandomVariableX,RandomVariableY,RandomVariableO) == 0 && CheckCollisionDetection(RandomVariableX,RandomVariableY,RandomVariableO) == 0 )
                    break ;
                end  

                end


                Distance = Inf*ones(1,length(tree.vertex));
                
                for j = 1:length(tree.vertex)
                  Distance(j) = sqrt( (RandomVariableX - tree.vertex(j).x)^2 + (RandomVariableY-tree.vertex(j).y)^2 + (deg2rad(RandomVariableO)-deg2rad(tree.vertex(j).o))^2 );
                end

                [val, ind] = min(Distance);

                NearestNodeX = tree.vertex(ind).x;
                NearestNodeY = tree.vertex(ind).y;
                NearestNodeOrientation = tree.vertex(ind).o;

                FoundAFeasiblePath = 1;
                NumberOfTries =0;
                            while true

                            [PathPoints,Index,OptimalVelocity,OptimalSteeringAngle] = BestPath(NearestNodeX,NearestNodeY,NearestNodeOrientation,RandomVariableX,RandomVariableY,RandomVariableO);

                            ObstacleFlag = 0;

                            for m = 1 : Index
                                  if ( IsThereAnObstacle(PathPoints(m,1),PathPoints(m,2)) == 1 || CheckCollisionDetectionCorners(PathPoints(m,1),PathPoints(m,2),PathPoints(m,3)) == 1 || CheckCollisionDetection(PathPoints(m,1),PathPoints(m,2),PathPoints(m,3)) == 1 )

                                     ObstacleFlag = 1;
                                     break ;
                                  end  
                            end

                            if ObstacleFlag == 0
                                break;
                            end

                            NumberOfTries = NumberOfTries + 1 ;

                            if NumberOfTries > 3
                                FoundAFeasiblePath = 0;
                                break;
                            end

                            end

            if FoundAFeasiblePath == 1
                break;
            end

            end


        NewX = PathPoints(Index,1);
        NewY = PathPoints(Index,2);
        NewO = PathPoints(Index,3);

        tree.vertex(iter).x = NewX; tree.vertex(iter).y = NewY; tree.vertex(iter).o = NewO;
        tree.vertex(iter).v = OptimalVelocity; tree.vertex(iter).s = OptimalSteeringAngle;

        tree.vertex(iter).PreviousX = tree.vertex(ind).x;
        tree.vertex(iter).PreviousY = tree.vertex(ind).y;
        tree.vertex(iter).PreviousOrientation = tree.vertex(ind).o;
        tree.vertex(iter).ind = iter; tree.vertex(iter).indPrev = ind;

        if sqrt( (NewX-RobotLocationXGoal)^2 + (NewY-RobotLocationYGoal)^2 ) <= MaxXYThreshold
            if abs(NewO - RobotOrientationGoal) < MaxOrientationThreshold
            plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
            break;
            end
        end

        plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'b');
        pause(0);

    end


    if iter >= MaxNumberOfIterations
        
                
        Statement = 'Found path to the point nearest to the goal';
        
        title({'KinoDynamic RRT';str;TimeElapsed;Statement});
        
    else
        Statement = 'Found path to the goal';
        
        title({'KinoDynamic RRT';str;TimeElapsed;Statement});

    end
    
    

        Distance = Inf*ones(1,length(tree.vertex));
        for j = 1:length(tree.vertex)
          Distance(j) = sqrt( (RobotLocationXGoal - tree.vertex(j).x)^2 + (RobotLocationYGoal-tree.vertex(j).y)^2 + (deg2rad(RobotOrientationGoal)-deg2rad(tree.vertex(j).o))^2 );
        end

          [val, ind] = min(Distance);

        NearestNodeX = tree.vertex(ind).x;
        NearestNodeY = tree.vertex(ind).y;
        NearestNodeOrientation = tree.vertex(ind).o;
 
        path.pos(1).x = NearestNodeX; path.pos(1).y = NearestNodeY; path.pos(1).o = NearestNodeOrientation;
        pathIndex = tree.vertex(ind).ind;
        j= -1;
        PathIndices = [pathIndex];
        
        while true
            pathIndex = tree.vertex(pathIndex).indPrev;
            PathIndices = union(PathIndices,pathIndex);
            
            if pathIndex == 1
                break;
            end
            
        end
        

        PI = 1;
       
        while 1
            path.pos(j+2).x = tree.vertex(PathIndices(PI)).x;
            path.pos(j+2).y = tree.vertex(PathIndices(PI)).y;
            path.pos(j+2).o = tree.vertex(PathIndices(PI)).o;
              PI = PI + 1;            
            
            H3 = DrawRobot(path.pos(j+2).x,path.pos(j+2).y,path.pos(j+2).o,'yellow');
            
            pause(0.2);

            if PI == length(PathIndices)+1
                break;
                
            else
                delete(H3);
            end

            j=j+1;

        end



        for j = 2:length(path.pos)

            plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'r', 'Linewidth', 3);

        end




    function H = DrawRobot(RobotLocation_X,RobotLocation_Y,Robot_Orientation,Color)
        
    RobotLocationXCorner1 = RobotLocation_X + (-15*cosd(Robot_Orientation)) - ( 5*sind(Robot_Orientation)) ; 
    RobotLocationYCorner1 = RobotLocation_Y + (-15*sind(Robot_Orientation)) + ( 5*cosd(Robot_Orientation)) ; 

    RobotLocationXCorner2 = RobotLocation_X + (15*cosd(Robot_Orientation)) - ( 5*sind(Robot_Orientation)) ; 
    RobotLocationYCorner2 = RobotLocation_Y + (15*sind(Robot_Orientation)) + ( 5*cosd(Robot_Orientation)) ; 

    RobotLocationXCorner3 = RobotLocation_X + (15*cosd(Robot_Orientation)) - ( -5*sind(Robot_Orientation)) ; 
    RobotLocationYCorner3 = RobotLocation_Y + (15*sind(Robot_Orientation)) + ( -5*cosd(Robot_Orientation)) ; 

    RobotLocationXCorner4 = RobotLocation_X + (-15*cosd(Robot_Orientation)) - ( -5*sind(Robot_Orientation)) ; 
    RobotLocationYCorner4 = RobotLocation_Y + (-15*sind(Robot_Orientation)) + ( -5*cosd(Robot_Orientation)) ; 

    RobotXVertices = [RobotLocationXCorner1 RobotLocationXCorner2 RobotLocationXCorner3 RobotLocationXCorner4 ];
    RobotYVertices = [RobotLocationYCorner1 RobotLocationYCorner2 RobotLocationYCorner3 RobotLocationYCorner4 ];

    H = patch(RobotXVertices,RobotYVertices,Color);
  
    end


    function [PathPoints,Index,OptimalVelocity,OptimalSteeringAngle] = BestPath(NearestNodeX,NearestNodeY,NearestNodeOrientation,RandomVariableX,RandomVariableY,RandomVariableO)

    DistanceThreshold = Inf;

    for i = 1 : NumberOfTrials

            if rand >= 0.5
                Speed = Velocity;
            else
                Speed = -Velocity;
            end

            SteeringAngle = (2*rand - 1)* SteeringAngleMax;

            TempX = NearestNodeX;
            TempY = NearestNodeY;
            TempO = NearestNodeOrientation;
            PathHistory = [];

            for r = 1 : (SimulationTime/SamplingInterval)

                TempX = TempX + (Speed*SamplingInterval*cosd(TempO));
                TempY = TempY + (Speed*SamplingInterval*sind(TempO));
                TempO = rad2deg(deg2rad(TempO) + (Speed*SamplingInterval/CarLength*deg2rad(SteeringAngle)));
                PathHistory = [PathHistory;[TempX TempY TempO]];

            end

            DistanceBetweenPathPointsFromRandomNumber = Inf*ones(1,length(PathHistory));        
            for k = 1:length(PathHistory)
              DistanceBetweenPathPointsFromRandomNumber(k) = sqrt( (RandomVariableX - PathHistory(k,1))^2 + (RandomVariableY-PathHistory(k,2))^2 + ((deg2rad(RandomVariableO) - deg2rad(PathHistory(k,3))))^2 );
            end

            [MinimumDistanceBetweenPathPointsFromRandomNumber, IndexNumber] = min(DistanceBetweenPathPointsFromRandomNumber);

            if MinimumDistanceBetweenPathPointsFromRandomNumber < DistanceThreshold

                DistanceThreshold = MinimumDistanceBetweenPathPointsFromRandomNumber;
                PathPoints = PathHistory;
                Index = IndexNumber;
                OptimalVelocity = Speed;
                OptimalSteeringAngle = SteeringAngle;


            end

    end

    end

end



