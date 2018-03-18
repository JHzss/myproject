//
// Created by jh on 18-3-15.
//

#ifndef MYPROJECT_SLIDEWINDOW_H
#define MYPROJECT_SLIDEWINDOW_H

#include "myheader.h"
#include "KeyFrame.h"


class SlideWindow
{
public:
    typedef shared_ptr<SlideWindow> Ptr;
    //SlideWindow();
    inline static SlideWindow::Ptr creat(){ return SlideWindow::Ptr(new SlideWindow());}

    vector<KeyFrame::Ptr> slidewinKefs;
    int slideWindowsize;
};


#endif //MYPROJECT_SLIDEWINDOW_H
