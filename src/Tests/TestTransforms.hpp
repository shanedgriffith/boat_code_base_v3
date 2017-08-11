//
//  TestTransforms.hpp
//
//
//  Created by Shane Griffith on 8/11/17.
//  Copyright (c) 2017 Shane Griffith. All rights reserved.
//


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include <DataTypes/Camera.hpp>

class TestTransforms {
private:
public:
    TestTransforms();
    static void CheckBtwn(Camera& _cam);
    static void TestLocalization(Camera& _cam);
};










