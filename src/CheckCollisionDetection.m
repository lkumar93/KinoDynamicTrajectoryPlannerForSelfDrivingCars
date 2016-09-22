%
% THIS FILE CONTAINS THE IMPLEMENTATION OF COLLISION DETECTION OF THE CAR AT ITS EDGES REQUIRED BY
% KINODYNAMIC PROBABILISTIC ROADMAP (PRM) AND KINODYNAMIC RAPIDLY EXPLORING RANDOM TREES (RRT) 
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


function Collision = CheckCollisionDetection(xRand,yRand,oRand)

    XCorner1 = round(xRand + (-15*cosd(oRand)) - ( 5*sind(oRand))) ; 
    YCorner1 = round(yRand + (-15*sind(oRand)) + ( 5*cosd(oRand))) ; 

    XCorner2 = round(xRand + (15*cosd(oRand)) - ( 5*sind(oRand))) ; 
    YCorner2 = round(yRand + (15*sind(oRand)) + ( 5*cosd(oRand))) ; 

    XCorner3 = round(xRand + (15*cosd(oRand)) - ( -5*sind(oRand))) ; 
    YCorner3 = round(yRand + (15*sind(oRand)) + ( -5*cosd(oRand))) ; 

    XCorner4 = round(xRand + (-15*cosd(oRand)) - ( -5*sind(oRand))) ; 
    YCorner4 = round(yRand + (-15*sind(oRand)) + ( -5*cosd(oRand))) ; 

    
    Collision = 0;
    
     if( IsThereAnObstacle(XCorner1,YCorner1) == 1 || IsThereAnObstacle(XCorner2,YCorner2) == 1 || IsThereAnObstacle(XCorner3,YCorner3) == 1 || IsThereAnObstacle(XCorner4,YCorner4) == 1 )
        Collision = 1;
        return
     end
     
     if(LineObstacleChecker(XCorner1,YCorner1,XCorner2,YCorner2,0,0) == 1)
         Collision = 1;
        return
     end
     
     if(LineObstacleChecker(XCorner2,YCorner2,XCorner3,YCorner3,0,0) == 1)
         Collision = 1;
        return
     end
     
     if(LineObstacleChecker(XCorner3,YCorner3,XCorner4,YCorner4,0,0) == 1)
         Collision = 1;
        return
     end
     
     if(LineObstacleChecker(XCorner4,YCorner4,XCorner1,YCorner1,0,0) == 1)
         Collision = 1;
        return
     end
         
 end

     
