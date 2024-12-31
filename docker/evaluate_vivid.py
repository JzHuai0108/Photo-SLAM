
# evaluate PhotoSLAM on a dataset

import os
import argparse
import subprocess


vivid_seqs = ['outdoor_robust_day1', 'outdoor_robust_day2', 'outdoor_robust_night1', 'outdoor_robust_night2', 
              'indoor_aggresive_dark', 'indoor_aggresive_global', 'indoor_aggresive_local', 'indoor_robust_dark',
              'indoor_robust_global', 'indoor_robust_local', 'indoor_robust_varying', 'indoor_unstable_dark',
              'indoor_unstable_global', 'indoor_unstable_local']

def main():
    parser = argparse.ArgumentParser(description="Evaluate PhotoSLAM on the VIVID Shin dataset.")
    parser.add_argument("--dataset_dir", type=str, default='/root/datasets/VIVID', help="Path to the VIVID dataset folder")
    parser.add_argument("--result_dir", type=str, default='/root/results/VIVID', help="Path to the VIVID result folder")
    parser.add_argument("--photoslam_dir", type=str, default="~/Photo-SLAM", help="root path of the Photo-SLAM codebase")
    parser.add_argument("--modality", type=str, default='thermal', help="Thermal_vis, Thermal_fs, Thermal_shin, Thermal_naive or RGB")
    args = parser.parse_args()

    found_seqs = []
    for folder in os.listdir(args.dataset_dir):
        if folder in vivid_seqs:
            found_seqs.append(os.path.join(args.dataset_dir, folder))

    print(f'Found {len(found_seqs)} folders:')
    for i, s in enumerate(found_seqs):
        print(f'{i}: {s}')

    config_modality = 'thermal'
    if 'thermal' in args.modality.lower():
        config_modality = 'thermal'
    else:
        config_modality = 'visual'

    for seqdir in found_seqs:
        seqname = os.path.basename(seqdir)
        resultdir = os.path.join(args.result_dir, args.modality, seqname)
        log_file = os.path.join(args.result_dir, args.modality, f"{seqname}_log.txt")
        os.makedirs(resultdir, exist_ok=True)  # Ensure result directory exists

        cmd = f"{args.photoslam_dir}/bin/vivid_mono " \
              f"{args.photoslam_dir}/ORB-SLAM3/Vocabulary/ORBvoc.txt " \
              f"{args.photoslam_dir}/cfg/ORB_SLAM3/Monocular/VIVID/{config_modality}.yaml " \
              f"{args.photoslam_dir}/cfg/gaussian_mapper/Monocular/VIVID/{config_modality}.yaml " \
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

