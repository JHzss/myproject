//
// Created by jh on 18-3-12.
//

#ifndef MYPROJECT_INITIALIZER_H
#define MYPROJECT_INITIALIZER_H

#include "myheader.h"
#include "Frame.h"
#include "KeyFrame.h"

class Initializer {

public:
    typedef shared_ptr<Initializer> Ptr;
    Initializer();
    Initializer::Ptr creat(){ return Initializer::Ptr(new Initializer());}

};


#endif //MYPROJECT_INITIALIZER_H
