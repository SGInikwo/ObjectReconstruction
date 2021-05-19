REC3D_PATH=/Users/goldy/Dev/_Libraries/_alt/rec3D
TV_PATH=$1

cd
cd $TV_PATH
python3 -u $REC3D_PATH/python/plotvector.py tvl1/tvl1_energy.txt /tvl1/tvl1_energy.png Energy Iterations TVL1 > /dev/null
python3 -u $REC3D_PATH/python/plotvector.py tvl1/tvl1_error.txt tvl1/tvl1_error.png Difference Iterations TVL1 > /dev/null