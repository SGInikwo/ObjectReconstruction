//
// Created by MacBook2015 on 01/03/2021.
//

#include "rMain.h"

using namespace std;
wxBEGIN_EVENT_TABLE(rMain, wxFrame)
    EVT_BUTTON(10001, rMain::OnInButton)
    EVT_BUTTON(10002, rMain::OnOutButton)
    EVT_BUTTON(10003, rMain::OnStartButton)
    EVT_BUTTON(10004, rMain::OnButtonClicked)
wxEND_EVENT_TABLE()


rMain::rMain() : wxFrame(nullptr, wxID_ANY, "Object Reconstructor", wxPoint(30,30), wxSize(800,500))
{
    putenv("KMP_DUPLICATE_LIB_OK=TRUE");
    SetBackgroundColour(wxColour(67,70,75));
    wxString hi[4] = {".ply",".obj",".blend"};
    inButton = new wxButton(this, 10001, "Input Dir", wxPoint(300,90), wxSize(50,20));
    outButon = new wxButton(this, 10002, "Output Dir", wxPoint(300,110), wxSize(50,20));
    inText = new wxTextCtrl(this, wxID_ANY, "Input Dir", wxPoint(350,90), wxSize(200,20), wxTE_READONLY);
    outText = new wxTextCtrl(this, wxID_ANY, "Output Dir", wxPoint(350,110), wxSize(200,20), wxTE_READONLY);
    fileBox = new wxComboBox(this, wxID_ANY, "File Type",wxPoint((800/2)-(85/2),140), wxSize(85,25), 3, hi, wxCB_READONLY | wxTE_PROCESS_ENTER);
    startButton = new wxButton(this, 10003, "Start", wxPoint(450,140), wxSize(50,20));
    mList = new wxListBox(this, wxID_ANY, wxPoint(90,180), wxSize(600,250));
    blender = new wxCheckBox(this, wxID_ANY, "Open Blender", wxPoint(600,110), wxSize(110,20));
    setMask = new wxButton(this, 10004, "Set Mask", wxPoint(150,110), wxSize(100,20));
    maxPix = new wxTextCtrl(this, wxID_ANY, "1500", wxPoint(600,80), wxSize(70,20));

    fileBox->SetLabel("File format");

}

rMain::~rMain(){

}

//void rMain::OnInputClicked(wxMouseEvent& WXUNUSED(event))
//{
//    maxPix->Clear();
//}

void rMain::OnButtonClicked(wxCommandEvent &evt){
    /* When the mask button is pressed, first a dialog choose img, then the python file executed */
    wxFileDialog dlg(this, "Choose input directory", "/Users/goldy/Dev/_Libraries/_alt/rec3D/python", "","PNG and JPG Files (*.png;*.jpg) | *.png;*.jpg", wxFD_DEFAULT_STYLE | wxFD_FILE_MUST_EXIST);

    if(dlg.ShowModal() == wxID_OK){
        imPath = dlg.GetPath();
        evt.Skip();
    }

    std::ofstream out("imagePath.txt");
    out << imPath;
    out.close();

    string maskSt = "python3 /Users/goldy/Dev/_Libraries/_alt/rec3D/python/windowcapture.py";
    system(maskSt.c_str());
    evt.Skip();
}

void rMain::OnInButton(wxCommandEvent &evt) {
    /* Input button */
    wxDirDialog dlg(this, "Choose input directory", "/Users/goldy", wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST);

    if(dlg.ShowModal() == wxID_OK){
        inText->SetLabelText(dlg.GetPath());
        inPath = dlg.GetPath();
        evt.Skip();
    }
}

void rMain::OnOutButton(wxCommandEvent &evt) {
    /* Output button */
    wxDirDialog dlg(this, "Choose input directory", "/Users/goldy", wxDD_DEFAULT_STYLE);

    if(dlg.ShowModal() == wxID_OK){
        outText->SetLabelText(dlg.GetPath());
        outPath = dlg.GetPath();
        cout << "This is the output: " + outPath << endl;
        //mList->Append("Yeah");
        evt.Skip();
    }
}

void rMain::OnStartButton(wxCommandEvent &evt) {
    /* Start button pressed */
    if(fileBox->GetCurrentSelection() > -1 ) {
        finalPath = "bash resize.sh " + inPath + " " + outPath + " " + maxPix->GetValue();
        mList->AppendString("Starting");
        /* resuze.sh */
        system(finalPath.c_str());
        mList->AppendString("final " + finalPath);
        cout << "final " << inPath << endl;
        mList->AppendString("out " + outPath);
        cout << "out again " << outPath << endl;

        /* storing path to .ply and .txt for Rec3D.cpp*/
        string first = outPath + "/output/sfm/structure_raw.ply";
        string second = outPath + "/output/sfm/structure_filt.ply";
        string third = outPath + "/output/sfm/bbox_filt.txt";

        String rFirst = outPath + "/output/undistort";
        String rSecond = outPath + "/output/sfm";
        String rt = outPath + "/output/rec3D";

        string input = outPath + "/output/";
        std::ofstream out("output.txt");
        out << input;
        out.close();

        float abF = 0.3;
        double abD = 0.5;

        /* Masking the images in undistort folder */
        string undistortMask = "python3 -u /Users/goldy/Dev/_Libraries/_alt/rec3D/python/undistortMask.py " + outPath;
        system(undistortMask.c_str());
        aB = new AdjustBox(first, second, third, abF, abD);
        r3D1 = new Rec3D(rFirst, rSecond, rt, true, true, false, false, false, true);
        r3D2 = new Rec3D(rFirst, rSecond, rt, false, true, true, false, false, false);
        r3D3 = new Rec3D(rFirst, rSecond, rt, false, true, false, true, false, false);

        string lastPath = "bash sfm.sh " + outPath + "/output/rec3D";
        mList->AppendString("last " + lastPath);
        cout << "lastp " << lastPath << endl;
        system(lastPath.c_str());

        string mCFirst = outPath + "/output/rec3D/tvl1/tvl1.vtk";

        std::ifstream t("output.txt");
        string str((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());

        if(fileBox->GetStringSelection() == ".ply"){
            string mCSecond = outPath + "/output/rec3D/tvl1/tvl1_mesh.ply";
            mC = new MarchingCubes(mCFirst, mCSecond, toObj);
            string cpString = "mv " + str + "/rec3D/tvl1/tvl1_mesh.ply " + str;
            system(cpString.c_str());
        }
        if(fileBox->GetStringSelection() == ".obj") {
            string mCSecond = outPath + "/output/rec3D/tvl1/tvl1_mesh.obj";
            mC = new MarchingCubes(mCFirst, mCSecond, true);
            string cpString = "mv " + str + "/rec3D/tvl1/tvl1_mesh.obj " + str;
            system(cpString.c_str());
        }
        if(fileBox->GetStringSelection() == ".blend") {
            string mCSecond = outPath + "/output/rec3D/tvl1/tvl1_mesh.ply";
            mC = new MarchingCubes(mCFirst, mCSecond, toObj);

            string cpString = "rm " + str + "/rec3D/tvl1/tvl1_mesh.ply";
            system(cpString.c_str());

            string argString = "/Applications/Blender.app/Contents/MacOS/Blender --background --python saveBlend.py -- " + str;
            system(argString.c_str());
        }


        if (blender->GetValue() == true){
            if(fileBox->GetStringSelection() == ".ply"){
                string openString = str + "/tvl1_mesh.ply";
                string openBlend = "/Applications/Blender.app/Contents/MacOS/Blender --python openBlend.py -- " + openString;
                system(openBlend.c_str());
            }
            if(fileBox->GetStringSelection() == ".obj"){
                string openString = str + "/tvl1_mesh.obj";
                string openBlend = "/Applications/Blender.app/Contents/MacOS/Blender --python openBlend.py -- " + openString;
                system(openBlend.c_str());
            }
            if(fileBox->GetStringSelection() == ".blend"){
                string openString = str + "/tvl1_mesh.blend";
                string openBlend = "/Applications/Blender.app/Contents/MacOS/Blender " + openString;
                system(openBlend.c_str());
            }
        }

        mList->AppendString("Finished");
        evt.Skip();
    }
}

