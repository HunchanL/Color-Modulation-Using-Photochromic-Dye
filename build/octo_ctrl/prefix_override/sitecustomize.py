import sys
if sys.prefix == 'c:\\python38':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = 'C:\\Users\\lhc15\\OneDrive\\Research\\1P_Camouflage Octopus\\ros2\\install'
