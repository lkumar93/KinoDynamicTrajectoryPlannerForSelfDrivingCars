%
% THIS IS AN IMPLEMENTATION OF KINODYNAMIC PROBABILISTIC ROADMAP (PRM)
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


function KinoDynamicPRM
    
    clc;
    
    pause(0);
    
    figure;

    ShowMap;
    
    RobotLocationXInit = 250;
    RobotLocationYInit = 250;
    RobotOrientationInit = 0;

    RobotLocationXGoal = 50;
    RobotLocationYGoal = 50;
    RobotOrientationGoal = 90;
    
    MaxNumberOfPointsToBePlaced = 350;
    
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
    
    MaximumNumberOfNodes = 3;

    StepSize = 50;
    
    title('Select the Starting Position')
    
    while true;
        
        
     [RobotLocationXInit,RobotLocationYInit] =ginput(1);
     RobotOrientationInit = round((MaximumOrientation-MinimumOrientation)*rand + MinimumOrientation );
     
       if ( IsThereAnObstacle(RobotLocationXInit,RobotLocationYInit) == 0 && CheckCollisionDetectionCorners(RobotLocationXInit,RobotLocationYInit,RobotOrientationInit) == 0 && CheckCollisionDetection(RobotLocationXInit,RobotLocationYInit,RobotOrientationInit) == 0 )
          break ;
       else
           title({'The selected starting position is not a valid configuration.';' Select the starting position again'})           
       end 
       
    end
    

    RobotLocationX = RobotLocationXInit;
    RobotLocationY = RobotLocationYInit;
    RobotOrientation = RobotOrientationInit;

    H1 = DrawRobot(RobotLocationX,RobotLocationY,RobotOrientation,'cyan');
    
    pause(0);

    hold on;
    
     title('Select the Goal Position')
    
     while true;
        
        
     [RobotLocationXGoal,RobotLocationYGoal] =ginput(1);
     RobotOrientationGoal = round((MaximumOrientation-MinimumOrientation)*rand + MinimumOrientation );
       if ( IsThereAnObstacle(RobotLocationXGoal,RobotLocationYGoal) == 0 && CheckCollisionDetectionCorners(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal) == 0 && CheckCollisionDetection(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal) == 0 )
          break ;
       else
           title({'The selected goal position is not a valid configuration';' Select the goal position again'})           
       end 
       
     end
    
    title('KinoDynamic PRM')

    H2 = DrawRobot(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal,'magenta');
    
    pause(0);

    legend([H1 H2],{'Starting Position','Goal Position'},'Location','eastoutside');
    
    PRMGraph = PGraph(3);

    StartNode = [RobotLocationXInit;RobotLocationYInit;RobotOrientationInit];
    StartNodeID = PRMGraph.add_node(StartNode);
    PRMGraph.highlight_node(StartNodeID,'NodeSize',4,'NodeFaceColor','cyan');
    pause(0);
    Alternator = 0;
    
    
               
    tic
    for NumberOfSamplePoints = 2 : MaxNumberOfPointsToBePlaced

        str = sprintf('Number Of Iterations = %d ',NumberOfSamplePoints);
        TimeElapsed =  sprintf('Time Elapsed = %f seconds ',toc);
        
        title({'KinoDynamic PRM';str;TimeElapsed});
           while true
                while true

                    Alternator = Alternator + 1;

                     %Generate Random Number in Goal Biased Region once in 15 times

                            if mod(Alternator,7) == 0
                                            RX = (RobotLocationXGoal - (StepSize/2));
                                            RandX = (StepSize)*rand;

                                            if RX < MinimumX
                                                RX = MinimumX ;
                                            else
                                                if RX > (MaximumX - StepSize)
                                                    RX = MaximumX - StepSize;                                                  
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

                                            RY = (RobotLocationYGoal - (StepSize/2));
                                            RandY = (StepSize)*rand;

                                            if RY < MinimumY
                                                RY = MinimumY ;
                                            else
                                                if RY > (MaximumY - StepSize)
                                                    RY = MaximumY - StepSize;
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

                                            RO = (RobotOrientationGoal - (StepSize/2));
                                            RandO = (StepSize)*rand;

                                            if RO < MinimumOrientation
                                                RO = MinimumOrientation ;
                                            else
                                                if RO > (MaximumOrientation - StepSize)
                                                    RO = MaximumOrientation - StepSize;
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

                                 %Generate the Goal Node once in 4 times

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


                    RandomNode = [RandomVariableX;RandomVariableY;RandomVariableO];

                    [Distances,NeighborNodeID] = PRMGraph.distances(RandomNode);

                    if length(Distances) < MaximumNumberOfNodes
                        MaxNodes = length(Distances);
                    else
                        MaxNodes = MaximumNumberOfNodes ;
                    end

                    NumberOfNodesForWhichPathIsNotFound = 0;

                    for node = 1 : MaxNodes
                        if PRMGraph.coord(NeighborNodeID(node)) == RandomNode
                            continue;
                        end


                        NeighborNode = PRMGraph.coord(NeighborNodeID(node)) ;
                        NearestNodeX = NeighborNode(1);
                        NearestNodeY = NeighborNode(2);
                        NearestNodeOrientation = NeighborNode(3);
                        FoundAFeasiblePath = 1;
                        NumberOfTries = 0;

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

                        NewX = PathPoints(Index,1);
                        NewY = PathPoints(Index,2);
                        NewO = PathPoints(Index,3);

                        if FoundAFeasiblePath == 0 
                            NumberOfNodesForWhichPathIsNotFound = NumberOfNodesForWhichPathIsNotFound + 1;
                            continue;
                        end


                            NewNode = [NewX;NewY;NewO];
                            NewNodeID = PRMGraph.add_node(NewNode);
                            PRMGraph.highlight_node(NewNodeID,'NodeSize',4,'NodeFaceColor','cyan');
                            pause(0);

                            PRMGraph.add_edge(NewNodeID,NeighborNodeID(node)); % ,[OptimalVelocity;OptimalSteeringAngle]
                            PRMGraph.highlight_edge(NewNodeID,NeighborNodeID(node),'EdgeColor','blue','EdgeThickness',0.2);
                            pause(0);

                    end

                    if(NumberOfNodesForWhichPathIsNotFound < MaxNodes)
                        break;
                    end

           end

    end


    VertexClosestToGoal = PRMGraph.closest([RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal]);
    VertexClosestToStartingPoint = PRMGraph.closest([RobotLocationXInit,RobotLocationYInit,RobotOrientationInit]);

    if PRMGraph.component(VertexClosestToStartingPoint) ~= PRMGraph.component(VertexClosestToGoal)
        error(' Starting Node and Goal Node are not connected. Rerun the planner');
    end


    PRMGraph.goal(VertexClosestToGoal);
    AStarPath = AStar(PRMGraph,VertexClosestToStartingPoint, VertexClosestToGoal);
    PRMGraph.highlight_path(AStarPath, 'red', 2);
 
    hold off;

    figure;
    
    ShowMap;
    
    str = sprintf('Number Of Iterations = %d ',NumberOfSamplePoints);
    TimeElapsed =  sprintf('Time Elapsed = %f seconds ',toc);
        
    title({'Trajectory generated by KinoDynamic PRM'; str ;TimeElapsed});

    H3 = DrawRobot(RobotLocationXInit,RobotLocationYInit,RobotOrientationInit,'cyan');
    
    pause(0);
    
    
    H4 = DrawRobot(RobotLocationXGoal,RobotLocationYGoal,RobotOrientationGoal,'magenta');

    pause(0);
    
    legend([H3 H4],{'Starting Position','Goal Position'},'Location','eastoutside');
    
    PRMGraph.highlight_node(VertexClosestToStartingPoint,'NodeSize',4,'NodeFaceColor','blue');
    PRMGraph.highlight_node(VertexClosestToGoal,'NodeSize',4,'NodeFaceColor','blue');
    
    AStarPathLength = length(AStarPath);

    for PathPointIndex = 1 :  AStarPathLength

        RobotLocation = PRMGraph.coord(AStarPath(PathPointIndex));

        RobotLocationX = RobotLocation(1);
        RobotLocationY = RobotLocation(2);
        RobotOrientation = RobotLocation(3);

        H5 = DrawRobot(RobotLocationX,RobotLocationY,RobotOrientation,'yellow');
        
        pause(0.2);
        
        if PathPointIndex <  AStarPathLength 
            delete(H5);
        end
        

        

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
        
    function AStarPath = AStar(PRMGraph,VertexClosestToStartingPoint, VertexClosestToGoal)


        ClosedSet = [];

        OpenSet = [VertexClosestToStartingPoint] ;

        CameFrom = [];

        GoalID = VertexClosestToGoal;

        G_Cost(VertexClosestToStartingPoint) = 0;
        DistanceBetweenStartAndGoal = sqrt( (RobotLocationXInit - RobotLocationXGoal)^2 + (RobotLocationYInit-RobotLocationYGoal)^2 + (deg2rad(RobotOrientationInit)-deg2rad(RobotOrientationGoal))^2 );
        H_Cost(VertexClosestToStartingPoint) = DistanceBetweenStartAndGoal;

        F_Cost(VertexClosestToStartingPoint) =  G_Cost(VertexClosestToStartingPoint) + H_Cost(VertexClosestToStartingPoint);

        while ~isempty(OpenSet)

            [DistanceValue,IndexOfTheNodeWithLowestFCost] = min(F_Cost(OpenSet));
            CurrentNodeID = OpenSet(IndexOfTheNodeWithLowestFCost);

            if CurrentNodeID == GoalID
                AStarPath = [];
                TempID = GoalID;

                while true
                    AStarPath = [TempID AStarPath];
                    TempID = CameFrom(TempID);
                    if TempID == 0
                        break;
                    end
                end

                return;

            end

           %Remove Current Node From The Open Set

           OpenSet = setdiff(OpenSet,CurrentNodeID);

           %Add Current Node To The Closed Set

           ClosedSet = union(ClosedSet,CurrentNodeID);

                 for Neighbour = PRMGraph.neighbours(CurrentNodeID)
                         if ismember(Neighbour, ClosedSet)
                             continue;
                         end
                         Tentative_G_Cost = G_Cost(CurrentNodeID) + ...
                            PRMGraph.distance(CurrentNodeID,Neighbour);

                         if ~ismember(Neighbour, OpenSet)
                             %add neighbor to openset
                             OpenSet = union(OpenSet, Neighbour);
                             H_Cost(Neighbour) = PRMGraph.distance(Neighbour, GoalID);
                             Tentative_Is_Better = true;
                         elseif Tentative_G_Cost < G_Cost(Neighbour)
                             Tentative_Is_Better = true;
                         else
                             Tentative_Is_Better = false;
                         end
                         if Tentative_Is_Better
                             CameFrom(Neighbour) = CurrentNodeID;
                             G_Cost(Neighbour) = Tentative_G_Cost;
                             F_Cost(Neighbour) = G_Cost(Neighbour) + H_Cost(Neighbour);
                         end
                 end

        end

        AStarPath = [];

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
    
        
        
        
    
    
