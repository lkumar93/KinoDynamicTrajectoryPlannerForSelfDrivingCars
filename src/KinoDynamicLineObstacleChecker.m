%
% THIS FILE CONTAINS THE IMPLEMENTATION OF AN OBSOLETE VERSION OF OBSTACLE DETECTION ALONG A LINE MODULE
% THAT WAS PREVIOUSLY BEING USED BY KINODYNAMIC PROBABILISTIC ROADMAP (PRM) AND KINODYNAMIC RAPIDLY
% EXPLORING RANDOM TREES (RRT) 
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

function Collision = KinoDynamicLineObstacleChecker(XCorner1,YCorner1,XCorner2,YCorner2,PathFlag,Orientation1,Orientation2)

Slope1 = (YCorner2-YCorner1)/(XCorner2 - XCorner1);

ConstantValue1 = YCorner1 - ( Slope1*XCorner1);

Collision = 0;
   
     x1 = XCorner1;
     y1 = YCorner1;

    if (abs(XCorner2 - XCorner1)>5)
        IncX1 = (XCorner2 - XCorner1) / 5;
    else

     if(XCorner1 < XCorner2)
        IncX1 = 1;
        else
            if (XCorner1 == XCorner2)
                IncX1 = 0 ;
            else 
                IncX1 = -1;
            end
     end
    end

     if (abs(YCorner2 - YCorner1) > 5)
         IncY1 = (YCorner2 - YCorner1)/5;
     else       
     if(YCorner1 < YCorner2)
        IncY1 = 1;
        else
            if (YCorner1 == YCorner2)
                IncY1 = 0 ;
            else 
                IncY1 = -1;
            end
     end
     end
     
     
     if( Slope1 == 0)   % horizontal line

            while ( x1 ~= XCorner2 )

                 if(IsThereAnObstacle(x1,y1) == 1)
                Collision = 1;
                return
                 end
                 
                 if(PathFlag == 1)
                     if(CheckCollisionDetection(x1,y1,Orientation) == 1 )
                        Collision = 1;
                        return
                     end
                 end
                 

            x1 = x1 + IncX1;


            end
          
      
     else
         
        if( Slope1 == -Inf || Slope1 == Inf || 1/Slope1 == 0)   % horizontal line
                        while ( y1 ~= YCorner2 )

                         if(IsThereAnObstacle(x1,y1) == 1)
                        Collision = 1;
                        return
                         end
                    
                if(PathFlag == 1)
                     if(CheckCollisionDetection(x1,y1,Orientation) == 1 )
                        Collision = 1;
                        return
                     end
                 end

                    y1 = y1 + IncY1;


                    end


         else
         
                 while(x1 ~= XCorner2)


                     y1 = (Slope1*x1) + ConstantValue1;

                     if(IsThereAnObstacle(x1,y1) == 1)
                         Collision = 1;
                         return
                     end
                     
                if(PathFlag == 1)
                     if(CheckCollisionDetection(x1,y1,Orientation) == 1 )
                        Collision = 1;
                        return
                     end
                 end

                     x1 = x1 + IncX1;

                 end
                 
                   x1 = XCorner1;
                   y1 = YCorner1;
                   
                   while ( y1 ~= YCorner2)


                    x1 = (y1 - ConstantValue1)*(1/Slope1) ;

                    if(IsThereAnObstacle(x1,y1) == 1)
                        Collision = 1;
                        return
                    end
                    
               if(PathFlag == 1)
                     if(CheckCollisionDetection(x1,y1,Orientation) == 1 )
                        Collision = 1;
                        return
                     end
                end

                    y1 = y1 + IncY1;

                   end
        end
     end
end
     
