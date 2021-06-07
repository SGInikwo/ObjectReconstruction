//
// Created by MacBook2015 on 01/03/2021.
//
#ifndef OBJRECON_RMAIN_H
#define OBJRECON_RMAIN_H
#include "wx/wx.h"
#include "stdio.h"
#include "stdlib.h"
#include "AdjustBox.h"
#include "Rec3D.h"
#include "MarchingCubes.h"


class rMain : public wxFrame
{
public:
    rMain();
    ~rMain();

public:
    void OnButtonClicked(wxCommandEvent &evt);
    void OnInButton(wxCommandEvent &evt);
    void OnOutButton(wxCommandEvent &evt);
    void OnFileBox(wxCommandEvent &evt);
    void OnStartButton(wxCommandEvent &evt);
    void OnInputClicked(wxCommandEvent &evt);
    void appendText(int num);

    wxButton* inButton = nullptr;
    wxButton* outButon = nullptr;
    wxButton* startButton = nullptr;
    wxButton* setMask = nullptr;
    wxCheckBox* blender = nullptr;
    wxComboBox* fileBox = nullptr;
    wxTextCtrl* maxPix = nullptr;
    wxTextCtrl* inText = nullptr;
    wxTextCtrl* outText = nullptr;
    wxTextCtrl* mText = nullptr;
    wxListBox* mList = nullptr;

    std::string imPath;
    std::string inPath;
    std::string outPath;
    std::string finalPath;

    bool toObj = false;

    AdjustBox* aB;
    Rec3D* r3D1;
    Rec3D* r3D2;
    Rec3D* r3D3;
    MarchingCubes* mC;
    wxDECLARE_EVENT_TABLE();
};


#endif //OBJRECON_RMAIN_H
