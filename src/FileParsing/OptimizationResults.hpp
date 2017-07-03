//
//  OptimizationResults.hpp
//  SIFTFlow
//
//  Created by Shane Griffith on 1/11/16.
//  Copyright © 2016 shane. All rights reserved.
//

#ifndef SRC_FILEPARSING_OPTIMIZATIONRESULTS_HPP_
#define SRC_FILEPARSING_OPTIMIZATIONRESULTS_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>

#include "FileParsing.hpp"

class OptimizationResults: public FileParsing{
protected:
    static const std::string boatfile;
    static const std::string velocityfile;
    static const std::string correspondencefile;
    static const std::string pointsfile;
    static const std::string reprofile;
    static const std::string statusfile;
    static const std::string drawdir;
    static const std::string parms;
public:

};

#endif /* SRC_FILEPARSING_OPTIMIZATIONRESULTS_HPP_ */
