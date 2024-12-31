
# evaluate PhotoSLAM on a dataset

import os
import argparse
import subprocess

rrxio_seqs = ['mocap_easy', 'gym', 'mocap_dark', 'indoor_floor', 'mocap_medium',
              'mocap_difficult', 'outdoor_campus', 'outdoor_street', 'mocap_dark_fast']

def main():
    parser = argparse.ArgumentParser(description="Evaluate PhotoSLAM on the RRXIO dataset.")
    parser.add_argument("--dataset_dir", type=str, default='/root/datasets/rrxio/irs_rtvi_datasets_2021', help="Path to the RRXIO dataset folder")
    parser.add_argument("--result_dir", type=str, default='/root/results/rrxio', help="Path to the RRXIO result folder")
    parser.add_argument("--photoslam_dir", type=str, default="~/Photo-SLAM", help="root path of the Photo-SLAM codebase")
    parser.add_argument("--modality", type=str, default='thermal_undistort', help="thermal_undistort or visual_undistort")
    args = parser.parse_args()

    found_seqs = []
    for folder in os.listdir(args.dataset_dir):
        if folder in rrxio_seqs:
            found_seqs.append(os.path.join(args.dataset_dir, folder))

    print(f'Found {len(found_seqs)} folders:')
    for i, s in enumerate(found_seqs):
        print(f'{i}: {s}')

    for seqdir in found_seqs:
        seqname = os.path.basename(seqdir)
        resultdir = os.path.join(args.result_dir, args.modality, seqname)
        log_file = os.path.join(args.result_dir, args.modality, f"{seqname}_log.txt")
        os.makedirs(resultdir, exist_ok=True)  # Ensure result directory exists

        cmd = f"{args.photoslam_dir}/bin/rrxio_mono " \
              f"{args.photoslam_dir}/ORB-SLAM3/Vocabulary/ORBvoc.txt " \
              f"{args.photoslam_dir}/cfg/ORB_SLAM3/Monocular/RRXIO/{args.modality}.yaml " \
              f"{args.photoslam_dir}/cfg/gaussian_mapper/Monocular/RRXIO/{args.modality}.yaml " \
              f"{seqdir} {resultdir} {args.modality} no_viewer"

        print(f"Running command\n{cmd}\nfor sequence {seqname}... Log: {log_file}")
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

