# How to use the bash shell to evaluate PhotoSLAM on a dataset

Use Replica as an example, follow the below steps.
## 1. copy docker/evaluate_replica.sh to the result dir, e.g.,
```
cp docker/evaluate_replica.py /media/$USER/ExtremeSSD/results
```

## 2. start docker with docker/run.sh, e.g.,
```
./run.sh $HOME/Documents/gauss_splat/Photo-SLAM /media/$USER/ExtremeSSD/datasets /media/$USER/ExtremeSSD/results
```

## 3. run in docker 
```
python3 /root/results/evaluate_replica.py  # process all sequences in the dataset
```
