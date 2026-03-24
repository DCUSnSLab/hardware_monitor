import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yeon/hardware_monitor/src/relay_bridge/install/relay_bridge'
