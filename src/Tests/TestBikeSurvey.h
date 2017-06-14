//
//  TestBikeSurvey.h
//  boat_code_base_v2
//
//  Created by Shane on 6/2/17.
//  Copyright © 2017 Shane. All rights reserved.
//

#ifndef TestBikeSurvey_h
#define TestBikeSurvey_h

class TestBikeSurvey{
protected:
    
    cv::Scalar ColorByDistance(double dist);
    cv::Scalar ColorByHeight(double z);
    
    
public:
    TestBikeSurvey(){}
    
    void TestTriangulation();
};



#endif /* TestBikeSurvey_h */
