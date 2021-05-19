REC3D_PATH=/Users/goldy/Dev/_Libraries/_alt/rec3D
CH_PATH=$1
F_PATH=$2
MAX_SIZE=$3

cd $CH_PATH
mkdir -p $F_PATH/output

echo "Starting 3D reconstruction ..."; echo;

# Resize images
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%% Resizing input images %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo;
mkdir -p $F_PATH/output/resize
rsync -av --exclude='output' --exclude='data.txt' . $F_PATH/output/resize > /dev/null
cd $F_PATH/output
python3 -u $REC3D_PATH/python/resizeq.py $MAX_SIZE

# SfM
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%% SfM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo;
SFM_METHOD=incremental
rm -f $F_PATH/output/resize/.DS_Store
python -u $REC3D_PATH/python/sfm.py $SFM_METHOD AKAZE_FLOAT ULTRA FASTCASCADEHASHINGL2 ESSENTIAL
mkdir -p undistort
mkdir -p sfm
mkdir -p rec3D
cp -r openmvg/$SFM_METHOD/sfm/* sfm
echo;