//
// Created by MacBook2015 on 01/03/2021.
//
#pragma once

#include "wx/wx.h"
#include "rMain.h"

#ifndef OBJRECON_RAPP_H
#define OBJRECON_RAPP_H

class rApp : public wxApp
{
public:
    rApp();
    ~rApp();

private:
    rMain* mFrame1 = nullptr;

public:
    virtual bool OnInit();
};


#endif //OBJRECON_RAPP_H
