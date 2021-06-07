//
// Created by MacBook2015 on 01/03/2021.
//

#include "rApp.h"

wxIMPLEMENT_APP(rApp);

rApp::rApp()
{
}

rApp::~rApp()
{
}

bool rApp::OnInit()
{
    mFrame1 = new rMain();
    mFrame1->Show();

    return true;
}