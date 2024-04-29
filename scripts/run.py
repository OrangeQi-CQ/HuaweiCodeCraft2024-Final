import subprocess
import concurrent.futures
import os
import json
import sys
import datetime
# from tqdm import tqdm
import argparse
import platform
import shutil
import time
import random

current_time = datetime.datetime.now().strftime("%m-%d %H-%M-%S)")

global judge_exe
global prog_exe

def run_single(maps, seed, print_flag, debug):
    map_name = maps[0]
    map_path = os.path.join(map_folder_path, map_name)
    
    arg = [judge_exe, prog_exe, '-m', map_path, '-l', 'NONE', '-r', '{}.rep'.format(map_name), '-d', 'INPUT.txt']
    print(arg)

    # if (debug):
        # arg.extend(['-f', '100000'])
    
    arg.extend(['-s', str(seed)])
    log_content = ''
    print(f"\033[91m{'*' * 10} {'stderr begin '.center(10)} {'*' * 10}\033[0m")
    result = subprocess.Popen(arg, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    seg_fault = False
    for line in result.stderr:
        if (line.strip() == "Segmentation fault"):
            seg_fault = True
        if print_flag:
            print(line, end='')
        log_content += line
    for line in result.stdout:
        output = line
        break
    print(f"\033[91m{'*' * 10} {'stderr end '.center(10)} {'*' * 10}\033[0m")
    if seg_fault:
        print("\033[1;31mSegmentation fault! seed: {}\033[0m".format(seed))
        # raise
    yield map_name, output, log_content


def test(maps: list, seed, print_flag, debug, log_flag=True):
    if log_flag:
        print("start run: \033[94m{}\033[0m".format(maps))
    total = 0
    if len(maps) == 1:
        run_func = run_single
    else:
        run_func = run_all
    
    start_time = time.time()

    for mapfile, output, log_content in run_func(maps, seed, print_flag, debug):
        name = current_time + " " + mapfile + " "
        if not os.path.exists('log'):
            os.makedirs('log')
        if log_flag:
            os.rename(os.path.join('replay', mapfile + '.rep'), os.path.join('replay', name + '.rep'))
            with open(os.path.join('log', name + '.log'), 'w') as log:
                log.write(log_content)

    end_time = time.time()
    
    if log_flag:
        print("\033[33mOutput:\t{}\033[0m".format(output))
        print("\033[33mTotal time:\t{:.1f}s\033[0m".format(end_time - start_time))
    return total

if __name__ == '__main__':
    sys.stdout = sys.stderr
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=int, default=0)
    parser.add_argument('--debug', type=bool, default=False)
    parser.add_argument('--seed', type=int, default=-1)
    parser.add_argument('--print', type=bool, default=True)
    
    args = parser.parse_args()
    
    if args.seed == -1:
        args.seed = random.randint(0, 1000000)
            
    print("random seed = {}".format(args.seed))
        
    if (args.debug):
        print("DEBUG MODE!")
        prog_exe = 'debug_'
    
    map_folder_path = os.path.abspath(os.path.join('judge', 'maps'))
    
    if args.map == 0:
        maps = os.listdir(map_folder_path)
        maps = sorted(maps, key=lambda x: (len(x), x))
    else:
        maps = ['map' + str(args.map) + '.txt']
        args.print = True

    system = platform.system()
    print('system: {}'.format(system))
    if system == 'Windows':
        working_directory = os.path.join('judge', 'windows')
        judge_exe = 'FinalJudge.exe'
        prog_exe = 'main.exe' if not args.debug else 'main_debug.exe'
    elif system == 'Linux':
        working_directory = os.path.join('judge', 'linux')
        judge_exe = './FinalJudge'
        prog_exe = './main' if not args.debug else './main_debug'
    else:
        working_directory = os.path.join('judge', 'mac')
        judge_exe = './FinalJudge'
        prog_exe = './main' if not args.debug else './main_debug'
        
    shutil.copy(prog_exe, os.path.join(working_directory, prog_exe))
    os.chdir(working_directory)
    test(maps, args.seed, args.print, args.debug)