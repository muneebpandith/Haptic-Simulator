import os, argparse
import allegro_kuka_slip27Jul as A
parser = argparse.ArgumentParser(description='ALLEGROHAND ON GRAVITY VALUES')
parser.add_argument('--g', type=float, help='floating value of the acc due to gravity (neg means towards ground)', default=-9.8)
args = parser.parse_args()

A.main(args.g)






