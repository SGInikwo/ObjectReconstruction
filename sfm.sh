# SfM
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%% SfM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo;
SFM_METHOD=global
rm /Users/macbook2015/Desktop/rec3D/data/indian/output/resize/.DS_Store
python -u $REC3D_PATH/python/sfm.py $SFM_METHOD AKAZE_FLOAT ULTRA FASTCASCADEHASHINGL2 ESSENTIAL
mkdir -p undistort
mkdir -p sfm
cp -r openmvg/$SFM_METHOD/sfm/* sfm
echo;