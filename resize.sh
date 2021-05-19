PATH=$1

cd $PATH
mkdir -p output

echo "Starting 3D reconstruction ..."; echo;

# Resize images
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%% Resizing input images %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo;
mkdir -p output/resize
rsync -av --exclude='output' --exclude='data.txt' . output/resize > /dev/null
cd output
python3 -u $REC3D_PATH/python/resize.py 700
