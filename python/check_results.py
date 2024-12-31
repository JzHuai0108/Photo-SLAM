
import os

def in_folder(folder, file_to_find):
    found = False
    for root, dirs, files in os.walk(folder):
        if file_to_find in files:
            file_path = os.path.join(root, file_to_find)
            # print(f"Found '{file_to_find}' at: {file_path}")
            found = True
            break

    if not found:
        print(f"'{file_to_find}' not found in '{folder}' or its subdirectories.")
    return found


def main(folder):
    logfiles = []
    for filename in os.listdir(folder):
        if filename.endswith('_log.txt'):
            logfiles.append(filename)
    for logfile in logfiles:
        logfile_path = os.path.join(folder, logfile)
        seq = logfile.replace('_log.txt', '')
        trajfile = os.path.join(folder, seq, "CameraTrajectory_TUM.txt")
        pc_file = "point_cloud.ply"
        found = in_folder(os.path.join(folder, seq), pc_file)
        if (os.path.isfile(trajfile) and
            os.path.getsize(trajfile) > 100 and
            found):
            status = True
        else:
            status = False
        with open(logfile_path, 'r') as log:
            lines = log.read().splitlines()  # Safely handle the file content
            last_line = lines[-1] if lines else ""  # In case the file is empty
            finish = False
            if "SLAM system exiting main" in last_line:
                finish = True
            else:
                print(f'{seq} system finish bad with last line: {last_line}')
            print(f"{seq} system finish: {finish}, result status: {status}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Check the results of PhotoSLAM in a directory for a dataset")
    parser.add_argument('folder', type=str, help="The folder to check, e.g., /home/pi/Desktop/photoslam_results/VIVID/RGB")
    args = parser.parse_args()
    main(args.folder)
