
# evaluate PhotoSLAM on a dataset

import os
import argparse
import subprocess


replica_seqs = ['office' + str(i) for i in range(5)] + ['room' + str(i) for i in range(3)]

def main():
    parser = argparse.ArgumentParser(description="Evaluate PhotoSLAM on the Replica dataset.")
    parser.add_argument("--dataset_dir", type=str, default='/root/datasets/Replica', help="Path to the Replica dataset folder")
    parser.add_argument("--result_dir", type=str, default='/root/results/Replica', help="Path to the Replica result folder")
    args = parser.parse_args()

    found_seqs = []
    for folder in os.listdir(args.dataset_dir):
        if folder in replica_seqs:
            found_seqs.append(os.path.join(args.dataset_dir, folder))

    print(f'Found {len(found_seqs)} folders:')
    for i, s in enumerate(found_seqs):
        print(f'{i}: {s}')

    for seqdir in found_seqs:
        seqname = os.path.basename(seqdir)
        resultdir = os.path.join(args.result_dir, seqname)
        log_file = os.path.join(resultdir, f"{seqname}_log.txt")
        os.makedirs(resultdir, exist_ok=True)  # Ensure result directory exists

        cmd = f"/Photo-SLAM/bin/replica_rgbd " \
              f"./ORB-SLAM3/Vocabulary/ORBvoc.txt " \
              f"./cfg/ORB_SLAM3/RGB-D/Replica/{seqname}.yaml " \
              f"./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml " \
              f"{seqdir} " \
              f"{resultdir} no_viewer"

        cmd_with_log = f"{cmd} > {log_file} 2>&1"
        print(f"Running command for sequence {seqname}... Log: {log_file}")
        with open(log_file, "w") as log:
            process = subprocess.Popen(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True
            )
            for line in process.stdout:
                print(line, end="")  # Print to standard output without buffering
                log.write(line)      # Write to the log file
            process.wait()  # Wait for the process to complete


if __name__ == "__main__":
    main()

