#coding=utf8
"""
Date:2020/7/7-
version：1.0
function: Solve the arc routing problem by Dantzig-Wolf decomposition.
Authors: Maocan Song.
"""
from Model import DW
import time
def main():
    start_time=time.time()
    mod=DW()
    mod.g_Dantzig_Wolf_decomposition_method()
    end_time=time.time()
    spend_time=end_time-start_time
    mod.g_output_result(spend_time)
    print("Running time: {} min".format(spend_time/60))

if __name__=="__main__":
    main()